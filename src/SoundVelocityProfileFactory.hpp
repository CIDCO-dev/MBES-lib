#ifndef SOUNDVELOCITYPROFILEFACTORY_HPP
#define SOUNDVELOCITYPROFILEFACTORY_HPP

class SoundVelocityProfileFactory{
	public:
		static SoundVelocityProfile * buildSaltWaterModel(){
			SoundVelocityProfile * svp = new SoundVelocityProfile();
			//TODO: set time/location?
			svp->add(0,1520);
			svp->add(15000,1520);
			return svp;
		}
};


#endif
