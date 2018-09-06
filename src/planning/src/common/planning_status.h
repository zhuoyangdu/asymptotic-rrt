//
// Created by zy on 18-9-6.
//

#ifndef PLANNING_COMMON_PLANNING_STATUS_H
#define PLANNING_COMMON_PLANNING_STATUS_H

#include <string>

namespace planning {

enum ErrorCode {
    OK = 0,
    CRASH
};

class PlanningStatus {
 public:
    PlanningStatus(ErrorCode code = ErrorCode::OK) : code_(code) {

    }

    bool ok() const {
        return code_ == ErrorCode::OK;
    }

 private:
    ErrorCode code_;
    std::string msg_;
};
}

#endif //PLANNING_PLANNING_STATUS_H
