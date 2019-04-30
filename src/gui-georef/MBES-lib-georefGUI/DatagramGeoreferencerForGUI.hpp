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
 * \brief Datagramer Georeferencer class extention of the Datagram Processor class
 */
class DatagramGeoreferencerForGui : public DatagramGeoreferencer{
        public:
                 /**Create a datagram georeferencer*/
                DatagramGeoreferencerForGui( std::ofstream & outFileIn )
                    : outFile( outFileIn )
                {
                    outFile << std::setprecision(6);
                    outFile << std::fixed;
                }

                /**Destroy the datagram georeferencer*/
                virtual ~DatagramGeoreferencerForGui() {}

        protected:

            virtual void processGeoreferencedPing(Eigen::Vector3d & georeferencedPing,uint32_t quality,uint32_t intensity);

        private:
            std::ofstream & outFile;


};

void DatagramGeoreferencerForGui::processGeoreferencedPing(Eigen::Vector3d & georeferencedPing,uint32_t quality,uint32_t intensity){
    outFile << georeferencedPing(0) << " " << georeferencedPing(1) << " " << georeferencedPing(2)
              << " " << quality  << " " << intensity << std::endl;
}

#endif
