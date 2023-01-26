#ifndef PID_Controller_h
#define PID_Controller_h

#include "Arduino.h"

namespace pid
{
    // The Controller Class requires a timing interval at a constant rate...
    // Use the Timer.h object in the loop() function...
    // Note: Not yet tested - ControlManager under development...
    template <typename real>
    class Controller
    {
    public:
        // Constructor
        Controller() = default;
        Controller(real Kp, real Ki, real Kd, real Ts);
        ~Controller() = default;

        // Getters
        real Error();
        real Integral();
        real Derivative();

        // Setters
        void SetPoint(real setPoint);

        // Methods
        real UpdatePID(real processPoint);

        // Strings
        String PrintError();
        String PrintIntegral();
        String PrintDerivative();
        String PrintEID();
        String PrintFpFiFd();

    private:
        // Inputs
        real m_setPoint; // desired value
        real m_measuredValue;
        // Errors
        real m_last_ef;
        // Proportional
        real m_proportional;
        // Integral
        real m_integral;
        // Derivative
        real m_derivative;

        // Controller Gains
        real m_Kp;
        real m_Ki;
        real m_Kd;

        // Sampled Time
        real m_Ts;

        // Private Methods

    };

    // Constructor

    template<typename real>
    Controller<real>::Controller(real Kp, real Ki, real Kd, real Ts)
    {
        // errors
        m_last_ef = (real)0;
        // integral
        m_integral = (real)0;
        // Controller Gains
        m_Kp = Kp;
        m_Ki = Ki;
        m_Kd = Kd;
        // Sampled Time
        m_Ts = Ts;
    }

    // Getters

    template<typename real>
    real Controller<real>::Error()
    {
        return m_last_ef;
    }

    template<typename real>
    real Controller<real>::Integral()
    {
        return m_integral;
    }

    template<typename real>
    real Controller<real>::Derivative()
    {
        return m_derivative;
    }

    // Setters

    template<typename real>
    void Controller<real>::SetPoint(real setPoint)
    {
        m_setPoint = setPoint;
    }

    // Methods

    template<typename real>
    real Controller<real>::UpdatePID(real measuredValue)
    {
        // PID Calculations
        // Studied several papers on PID controllers
        // and finally subtracted-out the most basic
        // elements on the subject... 

        // Error Function
        real ef = m_setPoint - measuredValue;

        // Integral
        real integral = m_integral + ef * m_Ts;

        // Derivative
        real derivative = (ef - m_last_ef) / m_Ts;

        // Store Values for csv-file
        m_integral = integral;
        m_derivative = derivative;
        m_last_ef = ef;

        // Control Value
        return m_Kp * ef + m_Ki * integral + m_Kd * derivative;
    }

    // Strings

    template<typename real>
    String Controller<real>::PrintError()
    {
        String str = String(m_last_ef, 2);
        return str;
    }

    template<typename real>
    String Controller<real>::PrintIntegral()
    {
        String str = String(m_integral, 2);
        return str;
    }

    template<typename real>
    String Controller<real>::PrintDerivative()
    {
        String str = String(m_derivative, 2);
        return str;
    }

    template<typename real>
    String Controller<real>::PrintEID()
    {
        // Prints functions as csv data for plotting...
        String str = String(m_last_ef, 2) + "," +
            String(m_integral, 2) + "," +
            String(m_derivative, 2);
        return str;
    }

    template<typename real>
    String Controller<real>::PrintFpFiFd()
    {
        real Fp = m_Kp * m_last_ef;
        real Fi = m_Ki * m_integral;
        real Fd = m_Kd * m_derivative;

        // Prints functions as csv data for plotting...
        String str = String(Fp, 2) + "," +
            String(Fi, 2) + "," +
            String(Fd, 2);
        return str;
    }

    // Private Methods
}

#endif