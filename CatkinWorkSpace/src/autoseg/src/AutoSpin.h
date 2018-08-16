#ifndef AUTO_SPIN_
#define AUTO_SPIN_ 

class AutoSpin
{
	public:
		AutoSpin(Eigen::Vector3d cen, Eigen::Vector3d currPos, Eigen::Vector4d currOrient);
		~AutoSpin();

	private:
		Eigen::Vector3d center;

		Eigen::Vector3d currentPosition;
		Eigen::Vector4d currentOrientation;
};

#endif /* ifndef AUTO_SPIN_ */
