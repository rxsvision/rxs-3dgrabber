#pragma once
class BallCalibration
{
public:
	static Eigen::VectorXf fitBallRansac(CP cloud, string id);
	static Eigen::VectorXf fitBallRansac3D(CP cloud, string id);
	static Eigen::VectorXf fitBallMLS(CP cloud);
	static Eigen::VectorXf fitBall(CP cloud, string id);
	void addBall(VCP balls);
	Eigen::Matrix4f registrationTwo(CP source_cloud, CP target_cloud);
	void updateCalibration();

	CP Calibration(VCP clouds);

	void save(int num, string file_name);
	void load(int num, string file_name);
	VCP ball_center;
private:
	vector<Eigen::Matrix4f> transforms;
};

