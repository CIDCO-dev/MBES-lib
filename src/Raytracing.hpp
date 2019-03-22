#ifndef RAYTRACING_HPP
#define RAYTRACING_HPP

class Raytracing{
public:
    static void rayTrace(Eigen::Vector3d & raytracedPing,Ping & ping,SoundVelocityProfile & svp){
	//TODO: do actual raytracing. This is just for quick testing purposes
	CoordinateTransform::sonar2cartesian(raytracedPing,ping.getAlongTrackAngle(),ping.getAcrossTrackAngle(), (ping.getTwoWayTravelTime()/(double)2) * (double)1480 );
    }
};

#endif
