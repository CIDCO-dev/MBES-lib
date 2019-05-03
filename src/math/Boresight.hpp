#ifndef BORESIGHT_HPP
#define BORESIGHT_HPP

class Boresight{
	public:
		static void buildMatrix(Eigen::Matrix3d & M,Attitude & boresight){
			double ch = boresight.getCh();
			double sh = boresight.getSh();
			double cp = boresight.getCp();
			double sp = boresight.getSp();
			double cr = boresight.getCr();
			double sr = boresight.getSr();

			M << 	ch*cp , ch*sp*sr - sh*cr  , ch*sp*cr + sh*sr,
				sh*cp , sh*sp*sr + ch*cr  , sh*sp*cr - ch*sr,
				-sp   , cp*sr		  , cp*cr;
		}

};

#endif
