**ADR-0001-NonlinearPlant SuccessiveLinearized(MPC(Nonlinear))**



**Date:**   17.12.2025

**Status:** Completed



**Context:**

* Graph Based Modelling of Hybrid Electric Aircraft Thermal and Electrical Energy Management System.
* Nonlinear MPC implementation on Simulink of Graph based model
* Successive Linearizer function for Linearizing Graph based model at each Controller call.



**Decision:**

* For successive linearizer MVs of Nonlinear optimized soultion was converted into Arrays for feedin during linearization.



**Consequences:**

* Optimization is Succesfull and Timing is improved
* Optimization Above 10 Prediction Horizon model gets rarely stable.



**Implementation notes:**

* Code remarks are in each mat file.
* In Simulink continous time x of nonlinear plant is fed to successive linearizer at every Ts after ZOH discretization.



**Mat Files:**

* aircraft\_dynamics\_linear 		(Successive Linearizer)
* aircraftDynamicsCT 			(Nonlinear Plant)
* aircraftCostFcn 			(Cost Function)
* aircraftEqCon 			(Equality Constraint)
* aircraftInEqCon 			(Inequality Constraint)
* Nonlinearplant\_linearizedmpc\_mat 	(Main Script)
* P\_prop 				(Measured Disturbance profiles)
* Parameters 				(Measured Disturbance Profiles)
* u1 					(MV Array for Successive linearizer)
* u2 					(MV Array for Successive linearizer)
* u3 					(MV Array for Successive linearizer)
* u4 					(MV Array for Successive linearizer)



**Simulink Model:**

* Nonlinearplant\_linearizedmpc\_sim



