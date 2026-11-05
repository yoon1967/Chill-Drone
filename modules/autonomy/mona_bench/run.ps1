param([ValidateSet('Build','Check','Graph','Smoke','Capture','Infer','Inspect','Replay')][string]$Action='Check',
      [string]$Bag, [string]$Url='http://host.docker.internal:8010',
      [ValidateRange(1,3600)][int]$Seconds=30,
      [ValidateRange(0,5)][double]$GoalForwardM=0,
      [ValidateRange(0.05,1)][double]$PlayRate=1,
      [ValidateSet('cuda','cpu')][string]$DepthDevice='cuda',
      [ValidateSet('da_v2_metric_indoor_small','da_v2_metric_indoor_small_fp32','sc_depth_v2_nyu')][string]$DepthModel='da_v2_metric_indoor_small',
      [switch]$RecordDepth)
$ErrorActionPreference='Stop'
$repoRoot=(Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$outputRoot=Join-Path $repoRoot 'data/outputs/mona-bench'
New-Item -ItemType Directory -Force -Path $outputRoot | Out-Null
if ($Action -eq 'Build') {
    docker build --progress plain -t ssafy/mona-bench:noetic $PSScriptRoot
    exit $LASTEXITCODE
}
$modelRoot=Join-Path $outputRoot 'models'
New-Item -ItemType Directory -Force -Path $modelRoot | Out-Null
$dockerArgs=@('run','--rm','--gpus','all','--mount',"type=bind,source=$outputRoot,target=/output",
              '--mount',"type=bind,source=$modelRoot,target=/models,readonly")
if ($Action -eq 'Check') { $dockerArgs+=@('ssafy/mona-bench:noetic','check') }
elseif ($Action -eq 'Graph') { $dockerArgs+=@('ssafy/mona-bench:noetic','graph') }
elseif ($Action -eq 'Smoke') { $dockerArgs+=@('ssafy/mona-bench:noetic','python3','/opt/mona_runtime/smoke_scale.py') }
elseif ($Action -in @('Capture','Infer')) {
    $captureId=(Get-Date -Format 'yyyyMMdd_HHmmss')+'_'+[guid]::NewGuid().ToString('N').Substring(0,8)
    if ($Action -eq 'Capture') {
        # -i forwards the keyboard: Enter (or Ctrl+C) stops early and still saves; -Seconds is the cap.
        $dockerArgs+=@('-i','ssafy/mona-bench:noetic','python3','/opt/mona_runtime/capture_http.py',
                       '--url',$Url,'--seconds',"$Seconds",'--output',"/output/captures/$captureId",
                       '--stop-on-enter')
    } else {
        $dockerArgs+=@('ssafy/mona-bench:noetic','python3','/opt/mona_runtime/infer_http.py',
                       '--url',$Url,'--output',"/output/inference/$captureId",'--device',$DepthDevice,
                       '--model',$DepthModel)
    }
}
else {
    if (-not $Bag) { throw "$Action requires -Bag with a local ROS1 bag." }
    $resolvedBag=(Resolve-Path -LiteralPath $Bag).Path
    $dockerArgs+=@('--mount',"type=bind,source=$resolvedBag,target=/input.bag,readonly")
    if ($Action -eq 'Replay' -and $GoalForwardM -gt 0) {
        # Offline planner goal ahead of the first SLAM body pose; never a drone command.
        $dockerArgs+=@('-e',"MONA_GOAL_FORWARD_M=$([string]::Format([cultureinfo]::InvariantCulture,'{0}',$GoalForwardM))")
    }
    if ($Action -eq 'Replay') {
        # Slower playback lets CPU depth inference see more recorded frames; time stays bag time.
        $dockerArgs+=@('-e',"MONA_PLAY_RATE=$([string]::Format([cultureinfo]::InvariantCulture,'{0}',$PlayRate))",
                       '-e',"MONA_DEPTH_DEVICE=$DepthDevice",'-e',"MONA_DEPTH_MODEL=$DepthModel",
                       '-e',"MONA_RECORD=$(if ($RecordDepth) { 'full' } else { 'light' })")
    }
    $dockerArgs+='ssafy/mona-bench:noetic'
    if ($Action -eq 'Inspect') { $dockerArgs+=@('python3','/opt/mona_runtime/inspect_bag.py','/input.bag') }
    else { $dockerArgs+='/opt/mona_runtime/replay.sh' }
}
& docker @dockerArgs
exit $LASTEXITCODE
