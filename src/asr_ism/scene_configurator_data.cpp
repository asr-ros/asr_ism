/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

// Local Include
#include "asr_ism/scene_configurator_data.hpp"

// ISM Inlcude
#include <ISM/utility/GeometryHelper.hpp>



void SceneConfiguratorData::addEstimation(ISM::ObjectPtr object)
{
    boost::lock_guard<boost::mutex> lock{estimations_mutex_}; //Thread-safety

    //Look whether object has already been observed and update or add the estimation for this object.
    bool found = false;
    for (ObjectEstimationPtrs::iterator it = estimations_.begin(); it != estimations_.end(); ++it)
    {
        if (((*it)->object->type == object->type) && ((*it)->object->observedId == object->observedId)
            && (!enable_neighborhood_evaluation_ || ISM::GeometryHelper::sharedNeighborhoodEvaluation((*it)->object->pose, object->pose, neighbour_distance_threshold_, neighbour_angle_threshold_)))
        {
            (*it)->object = object;
            (*it)->in_view = true;
            found = true;
            break;
        }
    }

    if (!found)
    {
        estimations_.push_back(ObjectEstimationPtr(new ObjectEstimation(object)));
    }
}

void SceneConfiguratorData::resetInViewForAllEstimations()
{
    boost::lock_guard<boost::mutex> lock{estimations_mutex_}; //Thread-safety

    for (ObjectEstimationPtrs::iterator it = estimations_.begin(); it != estimations_.end(); ++it)
    {
        (*it)->in_view = false;
    }
}

void SceneConfiguratorData::setMarkedForAllEstimationsInView(bool marked)
{
    boost::lock_guard<boost::mutex> lock{estimations_mutex_}; //Thread-safety

    for (ObjectEstimationPtrs::iterator it = estimations_.begin(); it != estimations_.end(); ++it)
    {
        if ((*it)->in_view)
        {
            (*it)->marked = marked;
        }
    }
}

void SceneConfiguratorData::unmarkAllEstimations()
{
    boost::lock_guard<boost::mutex> lock{estimations_mutex_}; //Thread-safety

    for (ObjectEstimationPtrs::iterator it = estimations_.begin(); it != estimations_.end(); ++it)
    {
        (*it)->marked = false;
    }
}

ISM::ObjectSetPtr SceneConfiguratorData::getObjectSetFromMarkedEstimations()
{
    ISM::ObjectSetPtr set(new ISM::ObjectSet());

    boost::lock_guard<boost::mutex> lock{estimations_mutex_}; //Thread-safety

    for (ObjectEstimationPtrs::iterator it = estimations_.begin(); it != estimations_.end(); ++it)
    {
        if ((*it)->marked)
        {
            set->insert(ISM::ObjectPtr(new ISM::Object(*(*it)->object)));
        }
    }
    return set;
}

ISM::ObjectSetPtr SceneConfiguratorData::getObjectSetFromAllEstimations()
{
    ISM::ObjectSetPtr set(new ISM::ObjectSet());

    boost::lock_guard<boost::mutex> lock{estimations_mutex_}; //Thread-safety

    for (ObjectEstimationPtrs::iterator it = estimations_.begin(); it != estimations_.end(); ++it)
    {
        set->insert(ISM::ObjectPtr(new ISM::Object(*(*it)->object)));
    }
    return set;
}

ObjectEstimationPtrs SceneConfiguratorData::getAllEstimations()
{
    ObjectEstimationPtrs estimations;

    boost::lock_guard<boost::mutex> lock{estimations_mutex_}; //Thread-safety

    for (const ObjectEstimationPtr estimation : estimations_)
    {
        estimations.push_back(ObjectEstimationPtr(new ObjectEstimation(*estimation)));
    }
    return estimations;
}

void SceneConfiguratorData::clearAllEstimations()
{
    boost::lock_guard<boost::mutex> lock{estimations_mutex_}; //Thread-safety
    estimations_.clear();
}

void SceneConfiguratorData::clearUnmarkedEstimations()
{
    boost::lock_guard<boost::mutex> lock{estimations_mutex_}; //Thread-safety

    for (ObjectEstimationPtrs::iterator it = estimations_.begin(); it != estimations_.end(); )
    {
        if (!((*it)->marked))
        {
            it = estimations_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}
