#ifndef DATAGRAMGEOREFERENCERFORGUI_HPP
#define DATAGRAMGEOREFERENCERFORGUI_HPP

#include <Eigen/Dense>

#include <iostream> // Temp while using cout?
#include <iomanip> // Temp while using cout?

#include "../../DatagramGeoreferencer.hpp"
#include "../../Ping.hpp"
#include "../../Position.hpp"
#include "../../Attitude.hpp"
#include "../../Georeferencing.hpp"
#include "../../svp/SoundVelocityProfile.hpp"
#include "../../svp/SoundVelocityProfileFactory.hpp"
#include "../../datagrams/DatagramEventHandler.hpp"
#include "../../math/Interpolation.hpp"

/*!
 * \brief "Datagramer Georeferencer to an ostream" class, extention of the Datagramer Georeferencer class
 */
class DatagramGeoreferencerToOstream : public DatagramGeoreferencer{
        public:
                 /**Create a datagram georeferencer*/
                DatagramGeoreferencerToOstream( std::ostream & out )
                    : out( out )
                {
                    out << std::setprecision(6);
                    out << std::fixed;
                }

                /**Destroy the datagram georeferencer*/
                virtual ~DatagramGeoreferencerToOstream() {}

        protected:

            virtual void processGeoreferencedPing(Eigen::Vector3d & georeferencedPing,uint32_t quality,uint32_t intensity);

        private:
            std::ostream & out; // ostream: can be used for a file or for std::cout


};

void DatagramGeoreferencerToOstream::processGeoreferencedPing(Eigen::Vector3d & georeferencedPing,uint32_t quality,uint32_t intensity){
    out << georeferencedPing(0) << " " << georeferencedPing(1) << " " << georeferencedPing(2)
              << " " << quality  << " " << intensity << std::endl;
}

#endif
