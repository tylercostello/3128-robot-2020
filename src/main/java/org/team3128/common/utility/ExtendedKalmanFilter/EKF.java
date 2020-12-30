package org.team3128.common.utility.ExtendedKalmanFilter;
import java.util.ArrayList;
import org.ejml.simple.SimpleMatrix;


public class EKF {
	private SimpleMatrix x;
	private SimpleMatrix P;
	private SimpleMatrix Q;
	private SimpleMatrix H;
	private SimpleMatrix R;
	private SimpleMatrix I;
	private SimpleMatrix z_sensors;
	private double alvariance;
	private double arvariance;

	private double b;
	private double dt;
	private double dt_2;
	private double dt_3;
	private double dt_4;

	// class constructor
	// start positions, acceleration variances, width, time since last check,
	// sensor variances
	public EKF(double xStart, double yStart, double thetaStart, double vlStart, double vrStart, double alvariance,
			double arvariance, double b, double dt, double r1, double r2, double r3) {

		this.alvariance = alvariance;
		this.arvariance = arvariance;
		this.b = b;
		this.dt = dt;
		this.dt_2 = dt * dt;
		this.dt_3 = dt_2 * dt;
		this.dt_4 = dt_3 * dt;

		double prev_time = 0;

		x = new SimpleMatrix(5, 1, true, new double[] { xStart, yStart, thetaStart, vlStart, vrStart });
		// Because we are giving it the correct starting values since it's an
		// EKF which requires them, we are very confident in the values
		P = SimpleMatrix.diag(0.01, 0.01, 0.01, 0.01, 0.01);
		// Chops off x and y coordinates because we can't measure those
		H = new SimpleMatrix(3, 5, true, new double[] { 0, 0, 1.0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 1.0 });
		I = SimpleMatrix.identity(5);

		z_sensors = new SimpleMatrix(3, 1);

		Q = new SimpleMatrix(5, 5);
		// Sensor variances
		R = SimpleMatrix.diag(r1, r2, r3);
	}

	// checks to see if 2 variables are close
	private boolean isClose(double in1, double in2) {
		if (Math.abs(in1 - in2) <= 0.001) {
			return true;
		}
		return false;
	}


	public double[] testFunction(double[] xInput) {
		double xNum = xInput[0];
		double y = xInput[1];
		double theta = xInput[2];
		double vl = xInput[3];
		double vr = xInput[4];
		double dt=xInput[5];

		//special case to check if vl and vr are about the same, because then r would go to infinity
		if (isClose(vl, vr)) {
			xNum = xNum + vl * Math.cos(theta) * dt;
			y = y + vl * Math.sin(theta) * dt;
			theta = theta;
		} else {
			double w = (vr - vl) / b;
			double r = (b / 2) * (vl + vr) / (vr - vl);
			xNum = r * Math.sin(theta) * Math.cos(w * dt) + r * Math.cos(theta) * Math.sin(w * dt) + xNum
					- r * Math.sin(theta);
			y = r * Math.sin(theta) * Math.sin(w * dt) - r * Math.cos(theta) * Math.cos(w * dt) + y
					+ r * Math.cos(theta);
			theta = theta + w * dt;
		}
		

		double[] returnArray = new double[5];
		returnArray[0] = xNum;
		returnArray[1] = y;
		returnArray[2] = theta;
		returnArray[3] = vl;
		returnArray[4] = vr;

		return returnArray;
	}



	// the function for calculating new position
	private SimpleMatrix aFunction(SimpleMatrix xInput, double b, double dt) {
		SimpleMatrix xOutput = new SimpleMatrix(5, 1);
		double xNum = xInput.get(0, 0);
		double y = xInput.get(1, 0);
		double theta = xInput.get(2, 0);
		double vl = xInput.get(3, 0);
		double vr = xInput.get(4, 0);

		//special case to check if vl and vr are about the same, because then r would go to infinity
		if (isClose(vl, vr)) {
			xNum = xNum + vl * Math.cos(theta) * dt;
			y = y + vl * Math.sin(theta) * dt;
			theta = theta;
		} else {
			double w = (vr - vl) / b;
			double r = (b / 2) * (vl + vr) / (vr - vl);
			xNum = r * Math.sin(theta) * Math.cos(w * dt) + r * Math.cos(theta) * Math.sin(w * dt) + xNum
					- r * Math.sin(theta);
			y = r * Math.sin(theta) * Math.sin(w * dt) - r * Math.cos(theta) * Math.cos(w * dt) + y
					+ r * Math.cos(theta);
			theta = theta + w * dt;
		}
		xOutput.set(0, 0, xNum);
		xOutput.set(1, 0, y);
		xOutput.set(2, 0, theta);
		xOutput.set(3, 0, vl);
		xOutput.set(4, 0, vr);

		return xOutput;

	}

	// main loop
	public double[] runFilter(double[] input) {

		z_sensors.set(0, 0, input[0]);
		z_sensors.set(1, 0, input[1]);
		z_sensors.set(2, 0, input[2]);
		this.dt = input[3];
		this.dt_2 = dt * dt;
		this.dt_3 = dt_2 * dt;
		this.dt_4 = dt_3 * dt;
		predict();
		update(z_sensors);

		double[] returnArray = new double[5];
		returnArray[0] = x.get(0, 0);
		returnArray[1] = x.get(1, 0);
		returnArray[2] = x.get(2, 0);
		returnArray[3] = x.get(3, 0);
		returnArray[4] = x.get(4, 0);

		return returnArray;

	}

	// predict step
	private void predict() {
		// sets q matrix
		x = aFunction(x, b, dt);
		Q.set(2, 0, 0);
		Q.set(2, 1, 0);
		Q.set(2, 2, (dt_4 * alvariance + dt_4 * arvariance) / (4 * b * b));
		Q.set(2, 3, (-dt_3 * alvariance) / (2 * b));
		Q.set(2, 4, (dt_3 * arvariance) / (2 * b));

		Q.set(3, 0, 0);
		Q.set(3, 1, 0);
		Q.set(3, 2, (-dt_3 * alvariance) / (2 * b));
		Q.set(3, 3, dt_2 * alvariance);
		Q.set(3, 4, 0);

		Q.set(4, 0, 0);
		Q.set(4, 1, 0);
		Q.set(4, 2, (dt_3 * arvariance) / (2 * b));
		Q.set(4, 3, 0);
		Q.set(4, 4, dt_2 * arvariance);

		SimpleMatrix A = new SimpleMatrix(5, 5);

		// set a matrix depending on if the velocities are close
		if (isClose(x.get(3, 0), x.get(4, 0))) {
			A.set(0, 0, 1);
			A.set(0, 1, 0);
			A.set(0, 2, -x.get(3, 0) * dt * Math.sin(x.get(2, 0)));
			A.set(0, 3, dt * Math.cos(x.get(2, 0)));
			A.set(0, 4, 0);

			A.set(1, 0, 0);
			A.set(1, 1, 1);
			A.set(1, 2, dt * x.get(3, 0) * Math.cos(x.get(2, 0)));
			A.set(1, 3, dt * Math.sin(x.get(2, 0)));
			A.set(1, 4, 0);

			A.set(2, 0, 0);
			A.set(2, 0, 0);
			A.set(2, 2, 1);
			A.set(2, 3, 0);
			A.set(2, 4, 0);

			A.set(3, 0, 0);
			A.set(3, 1, 0);
			A.set(3, 2, 0);
			A.set(3, 3, 1);
			A.set(3, 4, 0);

			A.set(4, 0, 0);
			A.set(4, 1, 0);
			A.set(4, 2, 0);
			A.set(4, 3, 0);
			A.set(4, 4, 1);
		} else {
			A.set(0, 0, 1);
			A.set(0, 1, 0);
			A.set(0, 2,
					(b * (x.get(4, 0) + x.get(3, 0))
							* (Math.cos(x.get(2, 0) + dt * (x.get(4, 0) - x.get(3, 0)) / b) - Math.cos(x.get(2, 0))))
							/ (2 * (x.get(4, 0) - x.get(3, 0))));
			A.set(0, 3,
					(-x.get(4, 0) * x.get(4, 0) * dt * Math.cos(x.get(2, 0) + dt * (x.get(4, 0) - x.get(3, 0)) / b)
							+ 2 * b * x.get(4, 0) * Math.sin(x.get(2, 0) + dt * (x.get(4, 0) - x.get(3, 0)) / b)
							- 2 * b * x.get(4, 0) * Math.sin(x.get(2, 0))
							+ x.get(3, 0) * x.get(3, 0) * dt
									* Math.cos(x.get(2, 0) + dt * (x.get(4, 0) - x.get(3, 0)) / b))
							/ (2 * (x.get(4, 0) - x.get(3, 0)) * (x.get(4, 0) - x.get(3, 0))));
			A.set(0, 4, (2 * x.get(3, 0) * b * Math.sin(x.get(2, 0))
					- 2 * x.get(3, 0) * b * Math.sin(x.get(2, 0) + dt * (x.get(4, 0) - x.get(3, 0)) / b)
					+ x.get(4, 0) * x.get(4, 0) * dt * Math.cos(x.get(2, 0) + dt * (x.get(4, 0) - x.get(3, 0)) / b)
					- x.get(3, 0) * x.get(3, 0) * dt * Math.cos(x.get(2, 0) + dt * (x.get(4, 0) - x.get(3, 0)) / b))
					/ (2 * (x.get(4, 0) - x.get(3, 0)) * (x.get(4, 0) - x.get(3, 0))));

			A.set(1, 0, 0);
			A.set(1, 1, 1);
			A.set(1, 2,
					(b * (x.get(4, 0) + x.get(3, 0))
							* (Math.sin(x.get(2, 0) + dt * (x.get(4, 0) - x.get(3, 0)) / b) - Math.sin(x.get(2, 0))))
							/ (2 * (x.get(4, 0) - x.get(3, 0))));
			A.set(1, 3,
					(x.get(3, 0) * x.get(3, 0) * dt * Math.cos(dt * (x.get(4, 0) - x.get(3, 0)) / b)
							* Math.sin(x.get(2, 0))
							+ x.get(3, 0) * x.get(3, 0) * dt * Math.sin(dt * (x.get(4, 0) - x.get(3, 0)) / b)
									* Math.cos(x.get(2, 0))
							+ 2 * b * x.get(4, 0) * Math.sin(dt * (x.get(4, 0) - x.get(3, 0)) / b)
									* Math.sin(x.get(2, 0))
							+ 2 * b * x.get(4, 0) * Math.cos(x.get(2, 0))
							- x.get(4, 0) * x.get(4, 0) * dt * Math.cos(dt * (x.get(4, 0) - x.get(3, 0)) / b)
									* Math.sin(x.get(2, 0))
							- x.get(4, 0) * x.get(4, 0) * dt * Math.sin(dt * (x.get(4, 0) - x.get(3, 0)) / b)
									* Math.cos(x.get(2, 0))
							- 2 * b * x.get(4, 0) * Math.cos(dt * (x.get(4, 0) - x.get(3, 0)) / b)
									* Math.cos(x.get(2, 0)))
							/ (2 * (x.get(4, 0) - x.get(3, 0)) * (x.get(4, 0) - x.get(3, 0))));
			A.set(1, 4,
					(x.get(4, 0) * x.get(4, 0) * dt * Math.cos(dt * (x.get(4, 0) - x.get(3, 0)) / b)
							* Math.sin(x.get(2, 0))
							+ x.get(4, 0) * x.get(4, 0) * dt * Math.sin(dt * (x.get(4, 0) - x.get(3, 0)) / b)
									* Math.cos(x.get(2, 0))
							+ 2 * b * x.get(3, 0) * Math.cos(dt * (x.get(4, 0) - x.get(3, 0)) / b)
									* Math.cos(x.get(2, 0))
							- x.get(3, 0) * x.get(3, 0) * dt * Math.cos(dt * (x.get(4, 0) - x.get(3, 0)) / b)
									* Math.sin(x.get(2, 0))
							- 2 * b * x.get(3, 0) * Math.sin(dt * (x.get(4, 0) - x.get(3, 0)) / b)
									* Math.sin(x.get(2, 0))
							- x.get(3, 0) * x.get(3, 0) * dt * Math.sin(dt * (x.get(4, 0) - x.get(3, 0)) / b)
									* Math.cos(x.get(2, 0))
							- 2 * b * x.get(3, 0) * Math.cos(x.get(2, 0)))
							/ (2 * (x.get(4, 0) - x.get(3, 0)) * (x.get(4, 0) - x.get(3, 0))));

			A.set(2, 0, 0);
			A.set(2, 1, 0);
			A.set(2, 2, 1);
			A.set(2, 3, -dt / b);
			A.set(2, 4, dt / b);

			A.set(3, 0, 0);
			A.set(3, 1, 0);
			A.set(3, 2, 0);
			A.set(3, 3, 1);
			A.set(3, 4, 0);

			A.set(4, 0, 0);
			A.set(4, 1, 0);
			A.set(4, 2, 0);
			A.set(4, 3, 0);
			A.set(4, 4, 1);

		}
		
		SimpleMatrix At = A.transpose();
		P = (A.mult(P.mult(At))).plus(Q);

	}

	// update loop
	private void update(SimpleMatrix z) {
		SimpleMatrix Y = z.minus(H.mult(x));
		SimpleMatrix Ht = H.transpose();
		SimpleMatrix S = (H.mult(P.mult(Ht))).plus(R);
		SimpleMatrix K = P.mult(Ht);
		SimpleMatrix Si = S.invert();
		K = K.mult(Si);
		x = x.plus(K.mult(Y));
		P = (I.minus(K.mult(H))).mult(P);
	}

}
