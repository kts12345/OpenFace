#define O2G_ERROR_SUCCESS                 0
#define O2G_ERROR_NO_MODEL_INFO           1
#define O2G_ERROR_ALREADY_INIT            2
#define O2G_ERROR_WRONG_PROFILE_NAME      3
#define O2G_ERROR_MODULE_NO_INIT          4
#define O2G_ERROR_NO_IMAGE                5
#define O2G_ERROR_READ_IMAGE              6
#define O2G_ERROR_PROFILE_NO_INIT         7
#define O2G_ERROR_FAIL_TO_MAKE_PROFILE    8
#define O2G_ERROR_ALREADY_EXIST_PROFILE   9
#define O2G_ERROR_INVALID_ROOT_PATH       10


/// 피팅 라이브러리 모듈 초기화
int profile_module_init(
  char* assert_root_path
);

/// 피팅 라이브러리 모듈 종료
int profile_module_final();

/// 이미지 제공
int profile_image_update_file(
  char* profile_name,
  char* image_temp_path,
  int face_outline_center_x,
  int face_outline_center_y
);

/// 이미지 제공 끝났음.
int profile_image_final(
  char* profile_name
);

/// 프로필 이미지 결과 저장
int profile_image_save(
  char* profile_name,
  int face_size_hint
);


