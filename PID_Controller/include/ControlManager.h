#ifndef Control_Manager_h
#define Control_Manager_h

#include "Arduino.h"

#include "Controller.h"
#include "RandomNumber.h"
#include "LinearMap.h"

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
        String PrintFunctionsPID();

    private:
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
        nmr::LinearMap<float> m_linearMap;

        // Private Method
        // Includes Both Random Values
        void m_SetRandomPoints();
        // Random Measured Value
        void m_SetRandomMeasuredValue();
    };

    // Constructor

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
        real Kp = (float)0.50;
        real Ki = (float)0.0;
        real Kd = (float)0.0;
        real Ts = sampledTime;
        m_controller = pid::Controller<float>(Kp, Ki, Kd, Ts);
        // Random Number Generator
        m_rNum = nmr::RandomNumber<float>(m_min, m_max);

        // Liner Map - MAX ought to be exclusive so 360 degrees ought to be exclusive
        m_linearMap = nmr::LinearMap<float>((float)min, (float)max, (float)0, (float)360);

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
        // Controller
        m_control = m_controller.UpdatePID(m_measuredValue);

        // Linear - Bounded
        // Simulate Max Plant Output -------------------------- BUGFIX
        if (m_control > (real)20) m_measuredValue += (real)20;
        else m_measuredValue += m_linearMap.Reverse(m_control);

        // Circular - Non-bounded
        // Polyninomial
    }
    //////////////////////////////////////////////////////////////

    // Private Methods

    template <typename real>
    void ControlManager<real>::m_SetRandomMeasuredValue()
    {
        real error = (real)0;
        do
        {
            m_measuredValue = m_linearMap.Map(m_rNum.Random());
            // m_setPoint 
            error = m_setPoint - m_measuredValue;
            // error squared
            error *= error;
            // error greater or equal to 10
        } while (error < (real)100);
    }

    template <typename real>
    void ControlManager<real>::m_SetRandomPoints()
    {
        m_setPoint = m_linearMap.Map(m_rNum.Random());
        m_SetRandomMeasuredValue();
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
        return m_control;
    }

    template <typename real>
    String ControlManager<real>::PrintFunctionsPID()
    {
        return m_controller.PrintFpFiFd();
    }
}

#endif
