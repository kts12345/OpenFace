#ifndef __O2G_MODULE_HPP__
#define __O2G_MODULE_HPP__

#include <string>
#include <vector>

#include "boost/filesystem.hpp"

#include "openface.hpp"

#include "o2g.h"


namespace o2g {

class O2gModule {
public:
  O2gModule() {
    _is_init = false;
    _model = nullptr;
  }
  void clear() {
    if (false == _is_init) return;

    _is_init = false;
    _root_path.clear();
    if (_model != nullptr) {
      openface::deinit_model(_model);
      _model = nullptr;
    }
  }
  int init(std::string root_path) {
    // 이미 초기화 되어 있음
    if (true == _is_init)
      return O2G_ERROR_SUCCESS;

    if (false == boost::filesystem::exists(root_path))
      return O2G_ERROR_INVALID_ROOT_PATH;

    std::string model_path = root_path;
    model_path += "/9_o2/fitting_info/model/main_clm_general.txt";
    //model_path = "/storage/emulated/0/o2/assets/09_o2/fitting_info/model/main_clm_general.txt";
    if (false == boost::filesystem::exists(model_path)) {
      return O2G_ERROR_NO_MODEL_INFO;
    }

    std::vector<std::string> arguments;
    arguments.push_back(root_path);
    arguments.push_back("-mloc");
    arguments.push_back(model_path);
    arguments.push_back("-gaze");
    _model = openface::init_model(arguments);;

    _root_path = root_path;

    _is_init = true;

    return O2G_ERROR_SUCCESS;
  }
  bool is_init() {
    return _is_init;
  }
  const std::string& root_path() {
    return _root_path;
  }
  void* model() {
    return _model;
  }

private:
  bool _is_init;
  std::string _root_path;
  void* _model;

};

} // end namespace o2g

#endif  // __O2G_MODULE_HPP__
