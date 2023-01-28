#ifndef Control_Manager_h
#define Control_Manager_h

#include "Arduino.h"

#include "Controller.h"
#include "RandomNumber.h"

namespace pid
{
    // (1) Controller Class should be accessed by the
    // ControlManager only and not by the main.cpp 
    // file...
    //
    // (2) The ControlManager has to be designed from
    // the ground-up for your power-plant...
    //
    // (3) Here the power-plant is simply a number-line
    // where the Controller chases a randomiazed
    // desired - set-point - until the actual
    // value catches up. Afterwards, a new random 
    // set-point begins another chase.
    //
    // Note: The number line can be either linear 
    // (bounded at both ends) or circular (non-
    // bounded allowing to cross the ends).
    template <typename real>
    class ControlManager
    {
    public:

        // Constructor
        ControlManager() = default;
        ControlManager(real sampledTime);
        ControlManager(real min, real max, real sampledTime);
        ~ControlManager() = default;

        // Getters
        // Setters
        void SetPoint(real setPoint);
        void MeasuredValue(real measuredValue);
        void MaxPlantOutput(real maxPlantOutput);

        // Methods
        void UpdateControlManager();

        // String
        String PrintSetPoint();
        String PrintMeasuredValue();
        String PrintErrorPID();
        String PrintIntegralPID();
        String PrintDerivativePID();
        String PrintControlPID();
        String PrintEID_PID();
        String PrintFunctionsPID();

    private:

        int m_counter;
        // Properties
        real m_min;
        real m_max;
        real m_error;
        real m_control;
        real m_setPoint;
        real m_measuredValue;
        real m_maxPlantOutput;
        pid::Controller<float> m_controller;
        nmr::RandomNumber<float> m_rNum;

        // Private Method
        // Includes Both Random Values
        void m_SetRandomPoints();
        // Random Measured Value
        void m_SetRandomMeasuredValue();
        // Power Plant Simulation
        void m_simulation();
        // Disturbance Simulation
        real m_disturbance();
    };

    // Constructor

    template <typename real>
    ControlManager<real>::ControlManager(real sampledTime)
    {
        m_counter = 0;
        // THIS IS NECESSARY TO UNDERSTAND THE MATH
        // Consistent data for
        // repeatable experiments...
        m_min = (real)0;
        m_max = (real)1000;
        // K-values domain: 0-1.0
        real Kp = (real)0.40;
        real Ki = (real)0.0;
        real Kd = (real)0.0;
        real Ts = sampledTime;

        // FIXED-CONSTANTS - USED FOR TESTING PID
        m_measuredValue = (real)330.9624876;
        m_setPoint = (real)20.7835093;
        // Switch data points
        // m_setPoint = (real)330.9624876;
        // m_measuredValue = (real)20.7835093;

        // Instantiate Object
        m_controller = pid::Controller<real>(Kp, Ki, Kd, Ts);
        m_controller.SetPoint(m_setPoint);
    }

    template <typename real>
    ControlManager<real>::ControlManager(real min, real max, real sampledTime)
    {
        m_setPoint = (real)0;

        // The min & max values are the end-points 
        // of the number-line.
        // Class properties
        m_min = min;
        m_max = max;
        // Controller Gains Kp, Ki and Kd should not be accessed from 
        // the main.cpp file..
        real Kp = (real)1;
        real Ki = (real)0.0;
        real Kd = (real)0.0;
        real Ts = sampledTime;
        m_controller = pid::Controller<real>(Kp, Ki, Kd, Ts);
        // Random Number Generator
        m_rNum = nmr::RandomNumber<real>(m_min, m_max);

        // Control Manager Points
        m_SetRandomPoints();
        // Controller SetPoint
        m_controller.SetPoint(m_setPoint);
    }

    // Getters
    // Setters

    template <typename real>
    void ControlManager<real>::SetPoint(real setPoint)
    {
        m_setPoint = setPoint;
    }

    template <typename real>
    void ControlManager<real>::MeasuredValue(real measuredValue)
    {
        m_measuredValue = measuredValue;
    }

    template <typename real>
    void ControlManager<real>::MaxPlantOutput(real maxPlantOutput)
    {
        m_maxPlantOutput = maxPlantOutput;
    }

    // Methods

    ///////////////////////////////////////////////////////////////
    template <typename real>
    void ControlManager<real>::UpdateControlManager()
    {
        // Controller - Control_U can be either (+) or (-)
        m_control = m_controller.UpdatePID(m_measuredValue);

        // WITH-OUT SIMULATION
        // Linear - Bounded
        m_measuredValue += m_control;

        // SIMULATION 
        // Linear - Bounded
        // m_simulation();

        // Circular - Non-bounded
        // Polyninomial
    }
    //////////////////////////////////////////////////////////////

    // Private Methods

    template <typename real>
    void ControlManager<real>::m_SetRandomMeasuredValue()
    {
        do
        {
            m_measuredValue = m_rNum.Random();
        } while (abs(m_setPoint - m_measuredValue) < (real)45);
    }

    template <typename real>
    void ControlManager<real>::m_SetRandomPoints()
    {
        m_setPoint = m_rNum.Random();
        m_SetRandomMeasuredValue();
    }

    template <typename real>
    void ControlManager<real>::m_simulation()
    {
        // Signed-Value for simulation...BUGFIX
        real sign = (real)1;
        if (m_control < (real)0) sign = (real)-1;

        // Linear - Bounded
        // Simulate Max Plant Output
        // Interesting that it has to be additive...BUGFIX
        // Use abs()...BUGFIX
        if (abs(m_control) > (real)20)
            m_measuredValue += (real)20 * sign + m_disturbance();
        else
            m_measuredValue += m_control * sign + m_disturbance();

    }

    template <typename real>
    real ControlManager<real>::m_disturbance()
    {
        m_counter++;
        // Constant Disturbances
        if (m_counter < 2 || m_counter > 7)
        {
            if ((m_counter % 2) == 0)
                return (real)-1.2;
            else
                return (real)1.2;
        }
        if (m_counter > 10) m_counter = 0;
        return (real)0;
    }

    // Strings

    template <typename real>
    String ControlManager<real>::PrintSetPoint()
    {
        String str = String(m_setPoint);
        return str;
    }

    template <typename real>
    String ControlManager<real>::PrintMeasuredValue()
    {
        String str = String(m_measuredValue);
        return str;
    }

    template <typename real>
    String ControlManager<real>::PrintErrorPID()
    {
        return m_controller.PrintError();
    }

    template <typename real>
    String ControlManager<real>::PrintIntegralPID()
    {
        return m_controller.PrintIntegral();
    }

    template <typename real>
    String ControlManager<real>::PrintDerivativePID()
    {
        return m_controller.PrintDerivative();
    }

    template <typename real>
    String ControlManager<real>::PrintControlPID()
    {
        String str = String(m_control);
        return str;
    }

    template <typename real>
    String ControlManager<real>::PrintEID_PID()
    {
        return m_controller.PrintEID();
    }

    template <typename real>
    String ControlManager<real>::PrintFunctionsPID()
    {
        return m_controller.PrintFpFiFd();
    }
}

#endif
