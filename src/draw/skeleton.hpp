#pragma once
#include <opencv2/core/types.hpp>
#include <utility>
#include <vector>

namespace draw {

enum KP {
  Pelvis = 0,
  L_Hip = 1,
  R_Hip = 2,
  Torso = 3,
  L_Knee = 4,
  R_Knee = 5,
  Neck = 6,
  L_Ankle = 7,
  R_Ankle = 8,
  L_BigToe = 9,
  R_BigToe = 10,
  L_SmallToe = 11,
  R_SmallToe = 12,
  L_Heel = 13,
  R_Heel = 14,
  Nose = 15,
  L_Eye = 16,
  R_Eye = 17,
  L_Ear = 18,
  R_Ear = 19,
  L_Shoulder = 20,
  R_Shoulder = 21,
  L_Elbow = 22,
  R_Elbow = 23,
  L_Wrist = 24,
  R_Wrist = 25,
  L_Pinky = 26,
  R_Pinky = 27,
  L_Middle = 28,
  R_Middle = 29,
  L_Index = 30,
  R_Index = 31,
  L_Thumb = 32,
  R_Thumb = 33
};

inline const int PARENTS[] = {
    -1, // 0: Pelvis
    0,  // 1: L_Hip
    0,  // 2: R_Hip
    0,  // 3: Torso
    1,  // 4: L_Knee
    2,  // 5: R_Knee
    3,  // 6: Neck
    4,  // 7: L_Ankle
    5,  // 8: R_Ankle
    7,  // 9: L_BigToe
    8,  // 10: R_BigToe
    7,  // 11: L_SmallToe
    8,  // 12: R_SmallToe
    7,  // 13: L_Heel
    8,  // 14: R_Heel
    6,  // 15: Nose
    15, // 16: L_Eye
    15, // 17: R_Eye
    15, // 18: L_Ear
    15, // 19: R_Ear
    6,  // 20: L_Shoulder
    6,  // 21: R_Shoulder
    20, // 22: L_Elbow
    21, // 23: R_Elbow
    22, // 24: L_Wrist
    23, // 25: R_Wrist
    24, // 26: L_Pinky
    25, // 27: R_Pinky
    24, // 28: L_Middle
    25, // 29: R_Middle
    24, // 30: L_Index
    25, // 31: R_Index
    24, // 32: L_Thumb
    25  // 33: R_Thumb
};

inline const std::vector<std::pair<int, int>> BONES = {
    {Pelvis, Torso}, {Torso, Neck}, {Neck, Nose}, {Nose, L_Eye}, {Nose, R_Eye}, {Nose, L_Ear}, {Nose, R_Ear}, {Pelvis, L_Hip}, {L_Hip, L_Knee}, {L_Knee, L_Ankle}, {L_Ankle, L_Heel}, {L_Ankle, L_BigToe}, {L_Ankle, L_SmallToe}, {L_BigToe, L_SmallToe}, {Pelvis, R_Hip}, {R_Hip, R_Knee}, {R_Knee, R_Ankle}, {R_Ankle, R_Heel}, {R_Ankle, R_BigToe}, {R_Ankle, R_SmallToe}, {R_BigToe, R_SmallToe}, {Neck, L_Shoulder}, {L_Shoulder, L_Elbow}, {L_Elbow, L_Wrist}, {L_Wrist, L_Thumb}, {L_Wrist, L_Index}, {L_Wrist, L_Middle}, {L_Wrist, L_Pinky}, {Neck, R_Shoulder}, {R_Shoulder, R_Elbow}, {R_Elbow, R_Wrist}, {R_Wrist, R_Thumb}, {R_Wrist, R_Index}, {R_Wrist, R_Middle}, {R_Wrist, R_Pinky}};

inline cv::Scalar GetColor(int index) {
  if (index == Pelvis || index == Torso || index == Neck || index == Nose)
    return cv::Scalar(0, 255, 0);
  bool isLeft = (index == L_Hip || index == L_Knee || index == L_Ankle || index == L_BigToe || index == L_SmallToe ||
                 index == L_Heel || index == L_Eye || index == L_Ear || index == L_Shoulder || index == L_Elbow ||
                 index == L_Wrist || index == L_Thumb || index == L_Index || index == L_Middle || index == L_Pinky);
  if (isLeft)
    return cv::Scalar(255, 100, 0);
  return cv::Scalar(0, 0, 255);
}

} // namespace draw