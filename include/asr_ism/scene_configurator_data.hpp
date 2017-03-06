/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Global Includes
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ISM Includes
#include <ISM/common_type/Object.hpp>
#include <ISM/common_type/ObjectSet.hpp>



/**
 *  ObjectEstimation struct represents an object estimation of an scene configuration.
 */
struct ObjectEstimation
{
    ISM::ObjectPtr object;
    bool marked;
    bool in_view;

    ObjectEstimation(ISM::ObjectPtr _object) : object(_object), marked(false), in_view(true) {}
    ObjectEstimation(const ObjectEstimation &other) : object(ISM::ObjectPtr(new ISM::Object(*(other.object)))), marked(other.marked), in_view(other.in_view) {}
};
typedef boost::shared_ptr<ObjectEstimation> ObjectEstimationPtr;
typedef std::vector<ObjectEstimationPtr> ObjectEstimationPtrs;



/**
 *  SceneConfiguratorData class holds data for current scene configuration.
 */
class SceneConfiguratorData
{
    public:

        SceneConfiguratorData(bool enable_nbrhood_eval = false, double nbr_angle_threshold = 0, double nbr_dist_threshold = 0) : enable_neighborhood_evaluation_(enable_nbrhood_eval), neighbour_angle_threshold_(nbr_angle_threshold), neighbour_distance_threshold_(nbr_dist_threshold) {}

        void addEstimation(ISM::ObjectPtr object);
        void resetInViewForAllEstimations();
        void setMarkedForAllEstimationsInView(bool marked);
        void unmarkAllEstimations();
        ISM::ObjectSetPtr getObjectSetFromMarkedEstimations();
        ISM::ObjectSetPtr getObjectSetFromAllEstimations();
        ObjectEstimationPtrs getAllEstimations();
        void clearAllEstimations();
        void clearUnmarkedEstimations();

    private:        

        ObjectEstimationPtrs estimations_;
        boost::mutex estimations_mutex_;

        bool enable_neighborhood_evaluation_;
        double neighbour_angle_threshold_;
        double neighbour_distance_threshold_;
};
