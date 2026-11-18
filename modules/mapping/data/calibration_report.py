"""Printable, local summary of a completed checkerboard calibration."""
from io import BytesIO
from pathlib import Path


def calibration_report_pdf(camera):
    from PIL import Image, ImageDraw, ImageFont

    summary = camera['calibration_summary']
    board = summary['board']
    fonts = (Path('C:/Windows/Fonts/malgun.ttf'),
             Path('/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc'),
             Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'))
    font_path = next((path for path in fonts if path.is_file()), None)
    korean = font_path is not None and font_path.name.lower() in {'malgun.ttf', 'notosanscjk-regular.ttc'}

    def label(ko, en):
        return ko if korean else en

    def font(size):
        return ImageFont.truetype(str(font_path), size) if font_path else ImageFont.load_default(size=size)

    image = Image.new('RGB', (1654, 2339), '#ffffff')
    draw = ImageDraw.Draw(image)
    dark, green, muted = '#18342a', '#315b42', '#5e7065'
    title, body, small = font(60), font(31), font(25)
    draw.rectangle((0, 0, 1654, 260), fill=dark)
    draw.text((110, 74), label('카메라 보정 결과', 'Camera Calibration Report'), font=title, fill='white')
    draw.text((110, 170), label('체크보드 보정  |  로컬 결과', 'Checkerboard calibration  |  Local result'),
              font=small, fill='#cde3d2')
    y = 330

    def section(name, rows):
        nonlocal y
        draw.text((110, y), name, font=font(38), fill=green)
        y += 72
        draw.line((110, y, 1544, y), fill='#d6e2d7', width=3)
        y += 32
        for label, value in rows:
            draw.text((120, y), label, font=small, fill=muted)
            draw.text((610, y), str(value), font=body, fill=dark)
            y += 70
        y += 52

    def number(value, places=3):
        return f'{float(value):.{places}f}'

    identity = str(camera.get('camera_identity') or 'Camera')
    if len(identity) > 48:
        identity = identity[:47] + '…'
    section(label('카메라', 'Camera'), [(label('카메라 / 입력', 'Name / source'), identity),
                       (label('해상도', 'Resolution'), f"{camera['width']} x {camera['height']} px"),
                       (label('보정 상태', 'Calibration'), label('체크보드 / 검증 통과', 'Checkerboard / validated')),
                       (label('생성 시각 (UTC)', 'Created (UTC)'), summary.get('created_at', '-')[:19].replace('T', ' '))])
    section(label('내부 파라미터 (픽셀)', 'Intrinsic parameters (pixels)'),
            [('fx / fy', f"{number(camera['fx'])} / {number(camera['fy'])}"),
                                             ('cx / cy', f"{number(camera['cx'])} / {number(camera['cy'])}"),
                                             ('k1 / k2', ' / '.join(number(v, 6) for v in camera['distortion'][:2])),
                                             ('p1 / p2', ' / '.join(number(v, 6) for v in camera['distortion'][2:4]))])
    section(label('수집 및 검증', 'Collection and validation'),
            [(label('안쪽 교차점', 'Inner corners'), f"{board['columns']} x {board['rows']}"),
             (label('한 칸 크기', 'Square size'), f"{number(board['square_mm'], 1)} mm"),
             (label('장면 수', 'Views'), label(
                 f"전체 {summary['views']} / 계산 {summary['training_views']} / 검증 {summary['validation_views']}",
                 f"{summary['views']} total / {summary['training_views']} fit / {summary['validation_views']} held out")),
             (label('계산 RMS', 'Fit RMS'), f"{number(summary['training_rms_px'])} px"),
             (label('검증 RMS', 'Validation RMS'), f"{number(summary['validation_rms_px'])} px")])
    draw.line((110, y, 1544, y), fill='#d6e2d7', width=3)
    y += 42
    for line in (label('JSON은 같은 카메라·해상도·초점·방향·크롭에서만 적용하세요.',
                       'Apply the JSON file to a matching camera, resolution, focus, orientation and crop.'),
                 label('이 PDF는 읽기·인쇄용이며 보정 프로필로 직접 가져올 수 없습니다.',
                       'This report is for reading and printing. It is not an importable camera profile.'),
                 label('낮은 재투영 오차가 실제 치수 정확도나 지도의 미터 단위를 보장하지 않습니다.',
                       'Low reprojection error does not establish physical accuracy or metric map scale.')):
        draw.text((110, y), line, font=font(22), fill=muted)
        y += 45
    output = BytesIO()
    image.save(output, format='PDF', resolution=200.0, title='Camera Calibration Report')
    return output.getvalue()
