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

                // TODO: warning: "'DatagramGeoreferencerForGui' has no out-of-line virtual method definitions; its vtable will emit in every translation unit"

                // TODO:DatagramGeoreferencerForGui will take "std::ifstream &outFile" as argument

                /**Create a datagram georeferencer*/
                DatagramGeoreferencerForGui( ){

                    // TODO: set precision on the stream

                    std::cout << std::setprecision(6);
                    std::cout << std::fixed;
                }

                /**Destroy the datagram georeferencer*/
                ~DatagramGeoreferencerForGui(){

                }



        protected:

		void processGeoreferencedPing(Eigen::Vector3d & georeferencedPing,uint32_t quality,uint32_t intensity){
            // TODO: output on the stream
            std::cout << "ForGUI" << georeferencedPing(0) << " " << georeferencedPing(1) << " " << georeferencedPing(2)
                      << " " << quality  << " " << intensity << std::endl;
		}


};

#endif
