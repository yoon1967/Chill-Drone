cmake_minimum_required(VERSION 3.16)
# Fail closed if the pinned upstream source changes. Never silently omit a guard.
set(source "${ORB_ROOT}/src/Sim3Solver.cc")
file(READ "${source}" content)
set(call "        ComputeSim3(P3Dc1i,P3Dc2i);")
string(REGEX MATCHALL "ComputeSim3\\(P3Dc1i,P3Dc2i\\)" calls "${content}")
list(LENGTH calls count)
if(NOT count EQUAL 2)
  message(FATAL_ERROR "Pinned Sim3Solver call sites changed")
endif()
string(REPLACE "${call}" "        if (!ComputeSim3(P3Dc1i,P3Dc2i)) continue;" content "${content}")
string(REPLACE "Eigen::Matrix4f bestSim3;" "Eigen::Matrix4f bestSim3 = Eigen::Matrix4f::Identity();" content "${content}")
string(FIND "${content}" "void Sim3Solver::ComputeSim3(" start)
string(FIND "${content}" "void Sim3Solver::CheckInliers()" end)
if(start LESS 0 OR end LESS start)
  message(FATAL_ERROR "Pinned Sim3Solver function boundaries changed")
endif()
string(SUBSTRING "${content}" 0 ${start} prefix)
string(SUBSTRING "${content}" ${end} -1 suffix)
file(WRITE "${source}" "#include \"safe_sim3.hpp\"\n${prefix}bool Sim3Solver::ComputeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2) {
    const bool valid = ssafy::compute_sim3(P1, P2, mbFixScale, mR12i, mt12i, ms12i, mT12i, mT21i);
    if (!valid) std::cerr << \"SSAFY_SIM3_REJECT degenerate_or_nonfinite_sample\" << std::endl;
    return valid;
}

${suffix}")
file(READ "${ORB_ROOT}/include/Sim3Solver.h" header)
string(REPLACE "    void ComputeSim3(" "    bool ComputeSim3(" header "${header}")
file(WRITE "${ORB_ROOT}/include/Sim3Solver.h" "${header}")

# On a camera-only stream gap, discard the motion prediction and try the old
# map first. The grace period starts at the returning frame, not during outage.
file(READ "${ORB_ROOT}/src/Tracking.cc" tracking)
set(anchor "    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpLastKeyFrame)")
string(FIND "${tracking}" "${anchor}" position)
if(position LESS 0)
  message(FATAL_ERROR "Pinned Tracking gap anchor changed")
endif()
string(REPLACE "${anchor}" "    if (mSensor == System::MONOCULAR && mCurrentFrame.mTimeStamp > mLastFrame.mTimeStamp + 0.75 && (mState == OK || mState == RECENTLY_LOST)) {
        mbVelocity = false;
        mState = RECENTLY_LOST;
        mTimeStampLost = mCurrentFrame.mTimeStamp;
    }
${anchor}" tracking "${tracking}")
string(REPLACE "mCurrentFrame.mTimeStamp-mTimeStampLost>3.0f && !bOK" "mCurrentFrame.mTimeStamp-mTimeStampLost>15.0f && !bOK" tracking "${tracking}")
file(WRITE "${ORB_ROOT}/src/Tracking.cc" "${tracking}")
