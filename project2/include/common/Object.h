#ifndef UNIFIED_MAKEFILE_PROJECT1_OBJECT_H
#define UNIFIED_MAKEFILE_PROJECT1_OBJECT_H

#include <vector>
#include "gfx/vec2.h"

class Object {
    public:
        virtual void drawObject() { return; };
        virtual std::vector<Vec2f> getPoints() { return {0}; };
};

#endif //UNIFIED_MAKEFILE_PROJECT1_OBJECT_H
