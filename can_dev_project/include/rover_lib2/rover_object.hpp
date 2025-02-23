#ifndef ROVER_OBJECT_HPP
#define ROVER_OBJECT_HPP

class RoverObject
{
  public:
    RoverObject() = default;

    virtual void init(void) = 0;
    virtual void update(void) = 0;
};

#endif  // ROVER_OBJECT_HPP
