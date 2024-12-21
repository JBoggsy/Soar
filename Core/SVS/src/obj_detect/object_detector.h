#pragma once

//SVS includes
#include "cliproxy.h"


class object_detector : public cliproxy {
public:
    object_detector();
    ~object_detector();



    void proxy_get_children(std::map<std::string, cliproxy*>& c);
    void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);
}
