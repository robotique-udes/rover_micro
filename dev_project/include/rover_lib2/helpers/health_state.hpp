#ifndef HEALTH_STATE_HPP
#define HEALTH_STATE_HPP

/**
 * @brief Singleton instance to manage the execution health state of a controller.
 * It must be global because assertions and each canbus manager should be able
 * to modify the health state of the controller. It doesn't make sense to create
 * multiple instances of it because it should be used as a controller wide
 * HealthState; if one part of the code goes into undefined behavior, the whole
 * controller is in undefined behavior.
 *
 * A different, not controller wide implementation should be created instead.
 */
class HealthState
{
  public:
    static HealthState& getInstance(void)
    {
        static HealthState instance_;
        return instance_;
    }

    void setInError(void)
    {
        _inError = true;
    }

    bool getInError(void)
    {
        return _inError;
    }

  private:
    HealthState() = default;
    HealthState(HealthState const&) = delete;
    void operator=(HealthState const&) = delete;

    bool _inError = false;
};

#endif  // HEALTH_STATE_HPP
