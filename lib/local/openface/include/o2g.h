#ifndef __O2G_HPP__
#define __O2G_HPP__

#ifdef _MSC_VER
#define O2FACE_CALLCONV __stdcall
#else
#define O2FACE_CALLCONV 
#endif

#ifdef __cplusplus
extern "C" {
#endif

  /// 피팅 라이브러리 api 입니다.
  ///
  /// 각각 모든 함수가 성공여부에 대해 O2G_ERROR_ 로 시작되는 에러값을 제공하고 있습니다.
  /// 리턴값을 반드시 확인하시고 리턴된 에러값을 참고해주시기 바랍니다.
  ///
  /// 일반적인 api 호출 순서는 아래와 같습니다.
  ///
  /// 1. profile_module_init : 피팅 모듈 초기화
  /// 2. profile_image_start : 동영상 사진 이미지 제공 시작
  /// 3. profile_image_update_file : 동영상 사진 이미지 제공
  /// 4. profile_image_final : 동영상 사진 이미지 제공 종료
  /// 5. profile_image_save : 프로필 저장
  /// 6. profile_module_final : 피팅 모듈 종료




  /// 피팅 라이브러리 에러값

  /// 성공
#define O2G_ERROR_SUCCESS                 0

/// asset_root_path 하위에 적절한 피팅라이브러리 초기화 정보가 없음
#define O2G_ERROR_NO_MODEL_INFO           1

/// profile_module_init 이 호출되었을 때, 이미 호출 되었음을 알림
#define O2G_ERROR_MODEL_ALREADY_INIT      2

/// profile_image_start, profile_image_update_file, profile_image_final, profile_image_save
/// 는 늘 동일한 profile_name  을 제공해야 하는데, 이 부분에 오류가 있음
#define O2G_ERROR_WRONG_PROFILE_NAME      3

/// profile_module_init 에 성공하지 않았는데 이후 다른 함수가 호출되었음
#define O2G_ERROR_MODULE_NO_INIT          4
  
/// profile_image_update_file 에서 제공한 사진 경로에 파일이 없음
#define O2G_ERROR_NO_IMAGE                5

/// profile_image_update_file 에서 제공한 사진 경로에 파일을 읽는데 실패했음
#define O2G_ERROR_READ_IMAGE              6

/// profile_image_start 에 성공하지 않고 이후 함수가 호출되었음
#define O2G_ERROR_PROFILE_NO_INIT         7

/// profile_image_final 에서 필요한 개수의 피팅을 위한 사진을 뽑는데 실패했음
#define O2G_ERROR_FAIL_TO_MAKE_PROFILE    8

/// profile_name 에 해당하는 프로필이 이미 존재함
#define O2G_ERROR_ALREADY_EXIST_PROFILE   9
 
/// asset_root_path 가 존재하지 않음
#define O2G_ERROR_INVALID_ROOT_PATH       10

/// 프로필 중 사진을 저장하는데 실패했음
#define O2G_ERROR_FAIL_TO_SAVE_IMAGE      11

/// profile_image_start 가 이미 호출되었음
#define O2G_ERROR_PROFILE_ALREADY_INIT    12







/// 피팅 라이브러리 모듈 초기화
///
/// 초기화가 완료된 다음에(함수 리턴 후) 다른 함수들 호출이 가능하다.
///
/// @returns : 모듈 초기화 성공 여부
///            O2G_ERROR_ 로 시작되는 에러코드 값
///
/// @param asset_root_path : o2g 라이브러리가 구동하기 위한 파일들의 root path
///                          하위에 fitting_info 디렉토리가 있다.
  int profile_module_init(
    char* asset_root_path
  );

  /// 피팅 라이브러리 모듈 종료
  /// 
  /// 이 함수가 호출된 이후에는 profile_module_init 외의 함수 호출은 실패한다.
  ///
  /// @returns : 항상 성공한다. O2G_ERROR_SUCCESS를 리턴한다.
  ///            이후 api 변경 없이 실패를 처리해야 할 경우를 대비하여 일단 int 리턴값을 유지한다.
  int profile_module_final();


  /// 프로필 만들기 시작
  ///
  /// @returns : 프로필을 저장할 수 있는 환경이 적절한지 여부
  ///
  /// @param profile_name : profile 이름
  ///                       1개 동영상이 촬영되는 동안 계속 같은 이름을 제공한다.
  /// @param camera_fov : camera 의 fov
  /// @param face_size_hint : 얼굴 크기 (대:0 / 중:1 / 소:2) 에 해당한다.
  int profile_image_start(
    char* profile_name,
    float camera_fov,
    int face_size_hint);

  /// 이미지 제공
  /// 
  /// 피팅을 위한 적절한 사진을 뽑기 위해,
  /// 초당 10개 정도 동영상 촬영되고 있는 사진을 라이브러리에서 계속 제공받는다.
  ///
  /// @returns : 사진을 image_temp_path 를 통해 잘 전달받았고, 이를 잘 처리했는지 여부
  ///            O2G_ERROR_ 로 시작되는 에러코드 값
  ///
  /// @param profile_name : profile 이름
  ///                       1개 동영상이 촬영되는 동안 계속 같은 이름을 제공한다.
  /// @param image_temp_path : 사진을 임의의 경로에 저장한다음 그 경로를 라이브러리에 제공한다.
  ///                          여기에는 그 임시 파일이 저장된 경로이다.
  ///                          이 함수가 리턴한 다음에는 해당 파일을 삭제하거나 업데이트해도 무방하다.
  ///                          따라서 일반적인 사용패턴은 profile_image_update_file 를 호출하는 동안
  ///                          계속 동일 파일에 동영상의 이미지를 저장하고
  ///                          profile_image_update_file를 호출하는 것이다.
  /// @param face_outline_center_x : 얼굴의 중심 x 좌표
  ///                                사용자에게 제공한 머리모양 가이드라인의 중심 위치를 제공
  ///                                피팅 라이브러리에서 최적화 등을 위해 사용됨.
  /// @param face_outline_center_y : 얼굴의 중심 y 좌표
  ///                                사용자에게 제공한 머리모양 가이드라인의 중심 위치를 제공
  ///                                피팅 라이브러리에서 최적화 등을 위해 사용됨.
  int profile_image_update_file(
    char* profile_name,
    char* image_temp_path,
    int face_outline_center_x,
    int face_outline_center_y
  );

  
  /// 이미지 제공 (메모리 파일 버전)
  /// 위의 profile_image_updatge_file 과 동일. 
  /// 단, 파일 경로가 아니라 메모리 buffer로 전달 받음
  int profile_image_update_memory_file(
    char* profile_name,
    char* image_buffer,
    int   image_buffer_size,
    int face_outline_center_x,
    int face_outline_center_y);


  /// 피팅 사진 제공 완료
  ///
  /// @returns : 안경 피팅에 사용하기 위한 사진이 모두 얻어졌는지 여부
  ///            O2G_ERROR_ 로 시작되는 에러코드 값
  ///
  /// @param profile_name : profile_image_update_file 에서 제공된 profile_name 과 동일
  int profile_image_final(
    char* profile_name
  );

  /// 프로필 이미지 결과 저장
  ///
  /// profile_module_init 에서 제공한 asset_root_path/profile/ 에 profile_name 이름으로 디렉토리를 만들고
  /// 하위에 피팅을 위한 사진 및 피팅 정보를 저장한다.

  /// @returns : 프로필 정보 저장 성공여부
  /// 
  /// @param profile_name : profile_image_update_file 에서 제공된 profile_name 과 동일
  int profile_image_save(
    char* profile_name);

  //#################### 디버그 함수들 ##############################//

  /// 이미지 제공하고 align 정보 얻기 
  int profile_get_pose(
    char* profile_name,
    char* image_buffer,
    int   image_buffer_size,
    int face_outline_center_x,
    int face_outline_center_y,
    float* tx, float* ty, float* tz,
    float* rx, float* ry, float* rz,
    float* fov);
#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // __O2G_HPP__
