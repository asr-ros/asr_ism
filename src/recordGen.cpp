/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/String.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ISM/common_type/Point.hpp>
#include <ISM/common_type/Track.hpp>
#include <ISM/common_type/Tracks.hpp>
#include <ISM/common_type/Object.hpp>
#include "ISM/recognizer/VotingSpace.hpp"
#include "ISM/recorder/Recorder.hpp"
#include <ISM/utility/TableHelper.hpp>
#include <ISM/utility/MathHelper.hpp>

#include <asr_ism/recordGenConfig.h>

#include <asr_ism_visualizations/ObjectModelVisualizerRVIZ.hpp>
#include "asr_ism_visualizations/VizHelperRVIZ.hpp"

#include <boost/filesystem/path.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>

//#include <eigen3/Eigen/Geometry.hpp>
#include <eigen3/unsupported/Eigen/Splines>
#include <Eigen/Geometry>

//#include "visStuff.hpp"

#include <boost/math/constants/constants.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
#include <random>
#include <iostream>
#include <fstream>
#include <sstream>

#include <math.h>
#include <stdint.h>
#include <tuple>


using visualization_msgs::MarkerArray;
using namespace VIZ;

double frand(double min, double max) {
	double f = (double) rand() / RAND_MAX;
	return min + f * (max - min);
}

enum class RecGenPointType
{
	Fix = 0,
	MidSpline = 1,
	MidSplineAlt = 2,
	Undefined = 3
};

namespace H
{
    template<class C, class T>
    auto contains(const C& v, const T& x)
    -> decltype(end(v), true)
    {
	return end(v) != std::find(begin(v), end(v), x);
    }
}

RecGenPointType recGenPointTypeFromString(const std::string& s)
{
	if(s == "Fix")
	{
		return RecGenPointType::Fix;
	} else if(s == "MidSpline")
	{
		return RecGenPointType::MidSpline;
	} else if(s == "MidSplineAlt")
	{
		return RecGenPointType::MidSplineAlt;
	}

	return RecGenPointType::Undefined;
}

enum class OrientCalcType
{
	Fix = 0,
    Random = 1,
    Linear = 2,
    Undefined = 3
};

OrientCalcType orientCalcTypeFromString(const std::string& s)
{
	if(s == "Fix")
	{
		return OrientCalcType::Fix;
	} else if (s == "Random")
	{
		return OrientCalcType::Random;
	} else if (s == "Linear")
	{
		return OrientCalcType::Linear;
	}
	return OrientCalcType::Undefined;
}

class RecordGenPoint
{
public:
	RecordGenPoint(std::string name, int32_t id, RecGenPointType type, double posX, double posY,
			double posZ, double orientW, double orientX, double orientY, double orientZ,
			OrientCalcType orientCalcType, double posErr, double orientDerivation, unsigned int from,
			unsigned int to, double linearDegX, double linearDegY, double linearDegZ):
		mTimestep(0),
		mCurrentApplied(false),
		mName(name),
		mId(id),
		mType(type),
		mCurrentPosX(posX),
		mCurrentPosY(posY),
		mCurrentPosZ(posZ),
		mCurrentOrientW(orientW),
		mCurrentOrientX(orientX),
		mCurrentOrientY(orientY),
		mCurrentOrientZ(orientZ),
	    mInitOrientW(orientW),
	    mInitOrientX(orientX),
	    mInitOrientY(orientY),
	    mInitOrientZ(orientZ),
		mOrientCalcType(orientCalcType),
		mPosErr(posErr),
		mOrientDer(orientDerivation),
		mFrom(from),
		mTo(to),
		mLinearDegX(linearDegX),
		mLinearDegY(linearDegY),
		mLinearDegZ(linearDegZ),
		mUniform(-1,1),
		mNormal(0, mOrientDer)

	{
		if(mOrientCalcType == OrientCalcType::Undefined)
		{
			throw std::runtime_error("Type of OrientCalc is undefined");
		}
		if(mType == RecGenPointType::Undefined)
		{
			throw std::runtime_error("Type of RecGenPoint is undefined");
		}
        mTrack.reset(new ISM::Track(name, boost::lexical_cast<std::string>(id)));
		std::random_device rd;
		mRandomGen.seed(rd());
		mRandomGenOrient.seed(rd());
		mVariantGen = new boost::variate_generator<boost::mt19937, boost::uniform_real<>>
				(mRandomGen, mUniform);
		mVarGenOrient = new boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
				(mRandomGenOrient, mNormal);
	}
	virtual ~RecordGenPoint()
	{
	}
	virtual bool isVisible() = 0;
	virtual void calcCurrObPos() = 0;
	virtual void calcCurrObOrient()
	{
		ISM::PointPtr currPos;
        if(mCurrentO && mCurrentO->pose)
		{
            currPos = mCurrentO->pose->point;
		}

		switch(mOrientCalcType)
		{
		case OrientCalcType::Fix:
		{
            mCurrentO.reset(new ISM::Object(mName,
                            new ISM::Pose(currPos,
                                    new ISM::Quaternion(mCurrentOrientW, mCurrentOrientX,
                                    		mCurrentOrientY, mCurrentOrientZ)),
                                    		boost::lexical_cast<std::string>(mId)));

			break;
		}
		case OrientCalcType::Random:
		{
			mCurrentOrientW = (*mVariantGen)();
			mCurrentOrientX = (*mVariantGen)();
			mCurrentOrientY = (*mVariantGen)();
			mCurrentOrientZ = (*mVariantGen)();

            mCurrentO.reset(new ISM::Object(mName,
                            new ISM::Pose(mCurrentO->pose->point,
                                    new ISM::Quaternion(mCurrentOrientW, mCurrentOrientX,
                                    		mCurrentOrientY, mCurrentOrientZ)),
                                    		boost::lexical_cast<std::string>(mId)));
			break;
		}
		case OrientCalcType::Linear:
		{
			if(mTimestep > 0 && mFrom < mTo)
			{
			    const double pi = boost::math::constants::pi<double>();
				double totSpan = mTo - mFrom;
				double currentLinEulX = mLinearDegX * ((double) mTimestep) / totSpan
						*pi / 180.0;
				double currentLinEulY = mLinearDegY * ((double) mTimestep) / totSpan
						*pi / 180.0;
				double currentLinEulZ = mLinearDegZ * ((double) mTimestep) / totSpan
						*pi / 180.0;

			    Eigen::Quaterniond linQ = Eigen::AngleAxisd(currentLinEulX, Eigen::Vector3d::UnitX())
			     * Eigen::AngleAxisd(currentLinEulY, Eigen::Vector3d::UnitY())
			     * Eigen::AngleAxisd(currentLinEulZ, Eigen::Vector3d::UnitZ());
			    Eigen::Quaterniond q(mInitOrientW, mInitOrientX, mInitOrientY, mInitOrientZ);
			    Eigen::Quaterniond totalQ = q * linQ;

			    mCurrentOrientW = totalQ.w();
			    mCurrentOrientX = totalQ.x();
			    mCurrentOrientY = totalQ.y();
			    mCurrentOrientZ = totalQ.z();
			}
			mCurrentO.reset(new ISM::Object(mName,
                    new ISM::Pose(mCurrentO->pose->point,
						new ISM::Quaternion(mCurrentOrientW, mCurrentOrientX,
								mCurrentOrientY, mCurrentOrientZ)),
								boost::lexical_cast<std::string>(mId)));
			break;
		}
		case OrientCalcType::Undefined:
		{
			throw std::runtime_error("OrientCalcType is Undefined");
			break;
		}
		}
	}
	virtual void forceInvisible(bool) = 0;
	virtual void apply()
	{
        if(!mCurrentO || !mCurrentO->pose->point ||
                !mCurrentO->pose->quat)
			throw "RecordGenPoint: Current Object must always be initialized";
		if(!mCurrentApplied)
		{
            auto point = mCurrentO->pose->point;
            auto quat = mCurrentO->pose->quat;
            point->eigen.x() = point->eigen.x() + (*mVariantGen)() * mPosErr;
            point->eigen.y() = point->eigen.y() + (*mVariantGen)() * mPosErr;
            point->eigen.z() = point->eigen.z() + (*mVariantGen)() * mPosErr;
            if(point->eigen.x() < -1.0 || point->eigen.x() > 1.0
                    || point->eigen.y() < -1.0 || point->eigen.y() > 1.0
                    ||point->eigen.z() < -1.0 || point->eigen.z() > 1.0 )
			{
				std::cerr << "Warning Point does not lie in range: " << point << std::endl
						<< "type: " << mName << " id: " << mId << std::endl
						<< "at timestep: " << mTimestep << std::endl;
				std::cin.ignore();
			}
			const double pi = boost::math::constants::pi<double>();

			Eigen::Matrix3d m;
			m = Eigen::AngleAxisd(pi * (*mVarGenOrient)(), Eigen::Vector3d::UnitX())
			 * Eigen::AngleAxisd(pi * (*mVarGenOrient)(), Eigen::Vector3d::UnitY())
			 * Eigen::AngleAxisd(pi * (*mVarGenOrient)(), Eigen::Vector3d::UnitZ());

			Eigen::Quaterniond errorQ(m);
			Eigen::Quaterniond q(quat->eigen.w(), quat->eigen.x(), quat->eigen.y(), quat->eigen.z());
	    Eigen::Quaterniond totalQ = q * errorQ;

	    quat->eigen.w() = totalQ.w();
            quat->eigen.x() = totalQ.x();
            quat->eigen.y() = totalQ.y();
            quat->eigen.z() = totalQ.z();
			/*
            quat->setW(quat->w);
            quat->setX(quat->x);
            quat->setY(quat->y);
            quat->setZ(quat->z);
			*/

            if(isVisible())
            {
		//TODO not in current ws
            	//mCurrentO->mTimestep = mTimestep;
                mTrack->objects.push_back(mCurrentO);
            }
            else
            {
                mTrack->objects.push_back(ISM::ObjectPtr());
                std::cout << mName << " is not visible" << std::endl;
            }

            mCurrentApplied = true;
		}
	}
	virtual void applyAndGotoNextTimestep()
	{
		apply();
		++mTimestep;
		mCurrentApplied = false;
		calcCurrObPos();
		calcCurrObOrient();
	}

	unsigned int mTimestep;
	bool mCurrentApplied;
	std::string mName;
	int32_t mId;
	ISM::TrackPtr mTrack;
    ISM::ObjectPtr mCurrentO;
    RecGenPointType mType;
	double mCurrentPosX;
	double mCurrentPosY;
	double mCurrentPosZ;
	double mCurrentOrientW;
	double mCurrentOrientX;
	double mCurrentOrientY;
	double mCurrentOrientZ;
	double mInitOrientW;
	double mInitOrientX;
	double mInitOrientY;
	double mInitOrientZ;
	OrientCalcType mOrientCalcType;
	double mPosErr;
	double mOrientDer;
	unsigned int mFrom;
	unsigned int mTo;
	double mLinearDegX;
	double mLinearDegY;
	double mLinearDegZ;
	boost::mt19937 mRandomGen;
	boost::mt19937 mRandomGenOrient;
	boost::uniform_real<> mUniform;
	boost::normal_distribution<> mNormal;
    boost::variate_generator<boost::mt19937, boost::uniform_real<>>* mVariantGen;
    boost::variate_generator<boost::mt19937, boost::normal_distribution<>>* mVarGenOrient;
    //ISM::PosePtr mCurrentPose;
};

class MidSplinePoint : public RecordGenPoint
{
private:
	void setCol(Eigen::MatrixXd &m, int col, Eigen::Vector3d &v, double time)
	{
		m(0, col) = time;
		m(1, col) = v.x();
		m(2, col) = v.y();
		m(3, col) = v.z();
	}
public:
	void calcSpline()
	{
		Eigen::Vector3d s = Eigen::Vector3d(mXs, mYs , mZs);
		Eigen::Vector3d e = Eigen::Vector3d(mXe, mYe , mZe);
		Eigen::Vector3d sToe = e - s;
        Eigen::Vector3d m = s + sToe/2;
        Eigen::Vector3d temp = sToe;
        temp.normalize();
        temp = temp * 2;
        Eigen::Vector3d orthSToe = temp.unitOrthogonal();
        orthSToe.normalize();
        orthSToe *= (sToe.norm()/2) * tan(mDegree * (boost::math::constants::pi<double>() / 180.0));
        Eigen::Vector3d p3 = m + orthSToe;

        //dimension, num points
        int numPoints = 3;
        double timeToNextPoint = mTimeSpan / (double) numPoints;
        Eigen::MatrixXd points(4, numPoints);
        setCol(points, 0, s, 0);
        setCol(points, 1, p3, timeToNextPoint);
        setCol(points, 2, e, mTimeSpan);
        mSpline = Eigen::SplineFitting<Eigen::Spline<double, 4>>::Interpolate(points, numPoints - 1);
	}
	MidSplinePoint(std::string name, int32_t id, double xs, double ys, double zs,
			double xe, double ye, double ze, double degree, double timeSpan,
			double orientW, double orientX, double orientY, double orientZ,
			OrientCalcType orientCalcType, double posErr, double orientErr,
			unsigned int from, unsigned int to,
			 double linearDegX, double linearDegY, double linearDegZ) :
				RecordGenPoint(name, id, RecGenPointType::MidSpline, xs, ys, zs, orientW,
						orientX, orientY, orientZ, orientCalcType, posErr,
						orientErr, from, to,
						linearDegX, linearDegY, linearDegZ),
						mIsInvisible(false),
                mXs(xs), mYs(ys), mZs(zs), mXe(xe), mYe(ye), mZe(ze), mDegree(degree), mTimeSpan(timeSpan)
	{
		calcSpline();
		calcCurrObPos();
		calcCurrObOrient();
	}

	void setNewParams(double xs, double ys, double zs,
			double xe, double ye, double ze, double degree, double timeSpan)
	{
		mXs = xs;
		mYs = ys;
		mZs = zs;

		mXe = xe;
		mYe = ye;
		mZe = ze;

		mDegree = degree;
		mTimeSpan = timeSpan;
		calcSpline();
		calcCurrObPos();
	}

	virtual ~MidSplinePoint()
	{
	}

	virtual bool isVisible()
	{
		return !mIsInvisible;
	}

	virtual void forceInvisible(bool b)
	{
		mIsInvisible = b;
	}

	virtual void calcCurrObPos()
	{
		ISM::QuaternionPtr currOrient;
        if(mCurrentO && mCurrentO->pose)
		{
            currOrient = mCurrentO->pose->quat;
		}

        Eigen::Vector4d values;
		if(mTimestep > mTimeSpan)
          values = mSpline(1);
		else
          values = mSpline(mTimestep/mTimeSpan);

		mCurrentPosX = values(1);
		mCurrentPosY = values(2);
		mCurrentPosZ = values(3);
        mCurrentO.reset(new ISM::Object(mName,
                        new ISM::Pose(new ISM::Point(mCurrentPosX, mCurrentPosY, mCurrentPosZ),
                                currOrient), boost::lexical_cast<std::string>(mId)));

	}
	bool mIsInvisible;
	double mXs;
	double mYs;
	double mZs;
	double mXe;
	double mYe;
	double mZe;
	Eigen::Spline<double, 4> mSpline;
	double mDegree;
	double mTimeSpan;
};

class MidSplinePointAlt : public MidSplinePoint
{
public:
	MidSplinePointAlt(std::string name, int32_t id, double xs, double ys, double zs,
			double xe, double ye, double ze, double degree, double timeSpan,
			double orientW, double orientX, double orientY, double orientZ,
			OrientCalcType orientCalcType, double posErr, double orientErr,
			unsigned int from, unsigned int to , double linearDegX, double linearDegY, double linearDegZ) :
          MidSplinePoint(name, id, xs, ys, zs,
                          xe, ye, ze, degree, timeSpan, orientW, orientX, orientY, orientZ,
                          orientCalcType, posErr, orientErr, from, to, linearDegX, linearDegY, linearDegZ)
    {

    }
	virtual void calcCurrObPos()
	{
		ISM::QuaternionPtr currOrient;
        if(mCurrentO && mCurrentO->pose)
		{
            currOrient = mCurrentO->pose->quat;
		}

        Eigen::Vector4d values;
        int timesTimeSpan = mTimestep / mTimeSpan;
		if((timesTimeSpan % 2) == 0)
          values = mSpline((mTimestep - (timesTimeSpan * mTimeSpan)) / mTimeSpan);
		else
          values = mSpline(1 - (mTimestep - (timesTimeSpan * mTimeSpan)) / mTimeSpan);

		mCurrentPosX = values(1);
		mCurrentPosY = values(2);
		mCurrentPosZ = values(3);
        mCurrentO.reset(new ISM::Object(mName,
                        new ISM::Pose(new ISM::Point(mCurrentPosX, mCurrentPosY, mCurrentPosZ),
                                currOrient), boost::lexical_cast<std::string>(mId)));

	}
};
class FixRecordGenPoint : public RecordGenPoint
{
public:
	FixRecordGenPoint(std::string name, int32_t id, double x, double y, double z,
			double orientW, double orientX, double orientY, double orientZ,
			OrientCalcType orientCalcType, double posErr, double orientErr,
			unsigned int from, unsigned int to, double linearDegX, double linearDegY, double linearDegZ):
		RecordGenPoint(name, id, RecGenPointType::Fix, x, y, z, orientW, orientX, orientY, orientZ,
				orientCalcType, posErr, orientErr, from, to, linearDegX, linearDegY, linearDegZ),
		mIsInvisible(false)
	{
		calcCurrObPos();
		calcCurrObOrient();
#if 0
        if(mTimestep > 0)
        {
        	unsigned int desTs = mTimestep;
        	mTimestep = 0;
        	forceInvisible(true);
            for(unsigned int i = 0; i < desTs; ++i)
            {
            	applyAndGotoNextTimestep();
            }
            forceInvisible(false);
        }
#endif
	}

	virtual ~FixRecordGenPoint()
	{
	}

	void setNewParams(double x, double y, double z)
	{
		mCurrentPosX = x;
		mCurrentPosY = y;
		mCurrentPosZ = z;
		calcCurrObPos();
	}

	virtual bool isVisible()
	{
		return !mIsInvisible;
	}

	virtual void forceInvisible(bool b)
	{
		mIsInvisible = b;
	}

	virtual void calcCurrObPos()
	{
		ISM::QuaternionPtr currOrient;
        if(mCurrentO && mCurrentO->pose)
		{
            currOrient = mCurrentO->pose->quat;
		}
        mCurrentO.reset(new ISM::Object(mName,
                        new ISM::Pose(new ISM::Point(mCurrentPosX, mCurrentPosY, mCurrentPosZ),
                                        currOrient), boost::lexical_cast<std::string>(mId)));

	}

	bool mIsInvisible;
};

using boost::filesystem::path;
typedef boost::shared_ptr<RecordGenPoint> RecordGenPointPtr;
typedef std::tuple<uint32_t,uint32_t,RecordGenPointPtr> FromToPointTuple;
namespace pt = boost::property_tree;

struct cmpFromToPointTuple {
    bool operator()(const FromToPointTuple& a, const FromToPointTuple& b) const {
    	if(std::get<2>(a)->mName == std::get<2>(b)->mName &&
    			std::get<2>(a)->mId == std::get<2>(b)->mId)
    	{
    		return std::get<1>(a) < std::get<0>(b);
    	} else
    	{
    		return (uintptr_t)(std::get<2>(a).get()) < (uintptr_t)(std::get<2>(b).get());
    	}
    }
};

class RecordGenerator
{
public:
	void parseXml(path xmlFile)
	{
		pt::ptree tree;

		pt::read_xml(xmlFile.string(), tree);

		for(auto& fstLvlNode : tree)
		{
			if(fstLvlNode.first == "object")
			{
				std::string name = fstLvlNode.second.get<std::string>("<xmlattr>.name");
				int32_t id = fstLvlNode.second.get<int32_t>("<xmlattr>.id");
                ISM::TrackPtr sharedTrack = ISM::TrackPtr(new ISM::Track(name,
                		boost::lexical_cast<std::string>(id)));
                mTracks.push_back(sharedTrack);
				for(auto& pointNode : fstLvlNode.second)
				{
                    if(pointNode.first == "point")
                    {
                        auto& pointAttrs = pointNode.second;

                        RecGenPointType pointType = recGenPointTypeFromString
                                        (pointAttrs.get<std::string>("<xmlattr>.type", ""));
                        int from = pointAttrs.get<uint32_t>("from");
                        int to = pointAttrs.get<uint32_t>("to");
                        if(from > to)
                        {
                        	std::stringstream ss;
                        	ss << "At: " << name << "," << id << " from must be less than to";
                        	throw std::runtime_error(ss.str());
                        }
                        double posX = pointAttrs.get<double>("posX", 0.0);
                        double posY = pointAttrs.get<double>("posY", 0.0);
                        double posZ = pointAttrs.get<double>("posZ", 0.0);
                        double orientW = pointAttrs.get<double>("orientW", 0.0);
                        double orientX = pointAttrs.get<double>("orientX", 0.0);
                        double orientY = pointAttrs.get<double>("orientY", 0.0);
                        double orientZ = pointAttrs.get<double>("orientZ", 0.0);
                        double xs = pointAttrs.get<double>("xs", 0.0);
                        double ys = pointAttrs.get<double>("ys", 0.0);
                        double zs = pointAttrs.get<double>("zs", 0.0);
                        double xe = pointAttrs.get<double>("xe", 0.0);
                        double ye = pointAttrs.get<double>("ye", 0.0);
                        double ze = pointAttrs.get<double>("ze", 0.0);
                        double degree = pointAttrs.get<double>("degree", 0.0);
                        double timeSpan = pointAttrs.get<double>("timeSpan", 0.0);
                        double posErr = pointAttrs.get<double>("posErr", 0.0);
                        double orientErr = pointAttrs.get<double>("orientErr", 0.0);
                        double linearDegX = pointAttrs.get<double>("linearDegX", 0.0);
                        double linearDegY = pointAttrs.get<double>("linearDegY", 0.0);
                        double linearDegZ = pointAttrs.get<double>("linearDegZ", 0.0);
                        OrientCalcType oCT = orientCalcTypeFromString(pointAttrs.get<std::string>("orientCalcType", ""));
                        std::vector<unsigned int> timestampVisible;
                        for(auto& pointAttr : pointAttrs)
                        {
                                if(pointAttr.first == "visible")
                                {
                                        std::string rangeString = pointAttr.second.get_value("");
                                        std::vector<std::string> range;
                                        boost::split(range, rangeString, boost::is_any_of("-"));
                                        if(range.size() == 2)
                                        {
                                                for(unsigned int i = boost::lexical_cast<unsigned int>(range[0]);
                                                                i <= boost::lexical_cast<unsigned int>(range[1]); ++i)
                                                {
                                                        timestampVisible.push_back(i);
                                                }
                                        } else
                                        {
                                                timestampVisible.push_back(boost::lexical_cast<unsigned int>(range[0]));
                                        }
                                }
                        }
                        RecordGenPointPtr p;
                        switch(pointType)
                        {
                        case RecGenPointType::Fix:
                        {
                                p = RecordGenPointPtr(new FixRecordGenPoint(name, id, posX, posY, posZ, orientW,
                                                orientX, orientY, orientZ, oCT, posErr, orientErr, from, to
                                                ,linearDegX, linearDegY, linearDegZ));
                                break;
                        }
                        case RecGenPointType::MidSpline:
                        {
                                p = RecordGenPointPtr(new MidSplinePoint(name, id, xs, ys, zs, xe, ye, ze, degree,
                                                timeSpan, orientW, orientX, orientY, orientZ, oCT, posErr, orientErr, from, to,
                                                 linearDegX, linearDegY, linearDegZ));
                                break;
                        }
                        case RecGenPointType::MidSplineAlt:
                        {
                                p = RecordGenPointPtr(new MidSplinePointAlt(name, id, xs, ys, zs, xe, ye, ze, degree,
                                        timeSpan, orientW, orientX, orientY, orientZ, oCT, posErr, orientErr, from, to,
                                         linearDegX, linearDegY, linearDegZ));
                                break;
                        }

                        default:
                                throw std::runtime_error("Error in parsing xml: RecGenPointType not valid");
                                break;
                        }
                        mTimestampVisibleXmlPoints[p] = timestampVisible;
                        p->mTrack = sharedTrack;
                        auto itToSucceed = mPoints.insert(std::make_tuple(from,to,p));
                        if(!itToSucceed.second)
                        {
							std::stringstream ss;
                        	ss << "At: " << name << "," << id << " point intervals overlapping";
                        	throw std::runtime_error(ss.str());
                        }
                    }
				}
			}

		}
	}
    void dynamicReconfCallback(asr_ism::recordGenConfig &config, uint32_t level)
    {
    	mShowFrom = config.from;
    	mShowTo = config.to;
    }

	RecordGenerator(std::string dbFile,
			std::string patternName = "pattern0", bool doPublish = false):
		mNextId(0),
		mTimestep(0),
		mDbFile(dbFile),
		mFilterViaVgOut(false),
		mPatternName(patternName),
		mShowCurrent(false),
		mDoPublish(doPublish),
		mShowFrom(0),
		mShowTo(0)
	{
            dynamic_reconfigure::Server<asr_ism::recordGenConfig>::CallbackType f = boost::bind(&RecordGenerator::dynamicReconfCallback, this, _1, _2);
	    mDynReconfServer.setCallback(f);
		rec.reset(new ISM::Recorder(mDbFile));
		//TODO drop tables?
        //rec->dropTables();
	}

	void filterViaVgOut(boost::filesystem::path vgOutFile)
	{
		mFilterViaVgOut = true;
		std::ifstream is(vgOutFile.string());
		std::string line;
		std::string type;
		std::string pattern;
		bool foundRep = true;
		while(std::getline(is, line))
		{
			if(line.find("Ref is:") != std::string::npos)
			{
				line.erase(line.begin(), line.begin() + line.find_first_of(":") + 1);
				mPatternToRef[pattern] = line;
			}
			if(line.find("Id for pattern:") != std::string::npos)
			{
				line.erase(line.begin(), line.begin() + line.find_first_of(":") + 1);
				pattern = line;
			} else if(line.find("Id for type:") != std::string::npos)
			{
				line.erase(line.begin(), line.begin() + line.find_first_of(":") + 1);
				type = line;
			} else if(line.find("These trackIds are in the same Orient Grid:") != std::string::npos)
			{
				foundRep = false;
			} else if(!foundRep)
			{
				uint32_t rep = 0;
				try
				{
                    rep = boost::lexical_cast<uint32_t>(line);
				} catch(boost::bad_lexical_cast& e)
                {
					continue;
                }
				mPatternTypeToVoxelReps[std::make_pair(pattern,type)].insert(rep);
				foundRep = true;
			}
		}
		for(auto& patternTypeToVoxelReps : mPatternTypeToVoxelReps)
		{
			std::string pattern = patternTypeToVoxelReps.first.first;
			std::string type = patternTypeToVoxelReps.first.second;
			for(uint32_t rep : patternTypeToVoxelReps.second)
			{
			    std::cout << "Rep for p: " << pattern << " t: " << type << " is: " << rep << std::endl;
			}
		}
	}

	void applyAndGotoNextTimestep()
	{
        for(auto& fromToPoint : mPoints)
        {
		    if(std::get<0>(fromToPoint) <= mTimestep &&
			    std::get<1>(fromToPoint) >= mTimestep)
		    {
			    auto& p = std::get<2>(fromToPoint);
			    std::vector<unsigned int>& visTs = mTimestampVisibleXmlPoints[p];

			    if(std::find(visTs.begin(), visTs.end(), mTimestep) != visTs.end())
			    {
				    p->forceInvisible(false);
			    } else
			    {
				    p->forceInvisible(true);
			    }
			    p->applyAndGotoNextTimestep();
		    }
        }
	    mTimestep++;
	}

	void apply()
	{
        for(auto& fromToPoint : mPoints)
        {
		    if(std::get<0>(fromToPoint) <= mTimestep &&
			    std::get<1>(fromToPoint) >= mTimestep)
		    {
			    auto& p = std::get<2>(fromToPoint);
			    std::vector<unsigned int>& visTs = mTimestampVisibleXmlPoints[p];

			    if(std::find(visTs.begin(), visTs.end(), mTimestep) != visTs.end())
			    {
				    p->forceInvisible(false);
			    } else
			    {
				    p->forceInvisible(true);
			    }
			    p->apply();
		    }
        }
	}

	void applyAndGotoNextTimestep(const uint32_t n)
	{
		for(unsigned int i = 0; i < n; ++i)
		{
			applyAndGotoNextTimestep();
		}
	}

	std::vector<ISM::ObjectPtr> getCurrentVisibleObjects()
    {
		std::vector<ISM::ObjectPtr> result;
		for(auto& fromToPoint : mPoints)
		{
		    if(std::get<0>(fromToPoint) <= mTimestep &&
			    std::get<1>(fromToPoint) >= mTimestep)
		    {
			    auto& p = std::get<2>(fromToPoint);
			    if(p->isVisible())
			    {
				    result.push_back(p->mCurrentO);
			    }
		    }
		}
		return result;
    }

	std::vector<ISM::ObjectPtr> getCurrentInvisibleObjects()
    {
		std::vector<ISM::ObjectPtr> result;
		for(auto& fromToPoint : mPoints)
		{
		    if(std::get<0>(fromToPoint) <= mTimestep &&
			    std::get<1>(fromToPoint) >= mTimestep)
		    {
			    auto& p = std::get<2>(fromToPoint);
			    if(!p->isVisible())
			    {
				    result.push_back(p->mCurrentO);
			    }
		    }
		}
		return result;
    }

	//void showFromTo(std_msgs::UInt32MultiArray::ConstPtr& msg)
	void showFromTo(const std_msgs::Float64MultiArray::ConstPtr& msg)
	{
	    mShowFrom = *msg->data.begin();
	    mShowTo = *(msg->data.begin() + 1);
	    mShowCurrent = false;
	}

    std::vector<ISM::ObjectPtr> getObjects(unsigned int from, unsigned int to)
    {
	std::vector<ISM::ObjectPtr> result;
    	static unsigned int oldTo = to;
    	if(!mDoPublish)
    	{
            if(to == oldTo)
            {
                    return result;
            } else
            {
                    oldTo = to;
            }
            ROS_INFO_STREAM("from is " << from << " << to: " <<to );
    	}
    	if(mTimestep < to)
    	{
    		//TODO
    		applyAndGotoNextTimestep(to - mTimestep);
    	}

    	if(mTimestep == to)
    	{
		    apply();
    	}
		if(from <= to)
		{
			for(auto& t : mTracks)
			{
				bool isRef = false;
                if(mFilterViaVgOut)
                {
                	std::vector<std::string> isRefInPatterns;
                	for(auto& patternToRef : mPatternToRef)
                	{
                		if(patternToRef.second == t->type)
                		{
                			isRefInPatterns.push_back(patternToRef.first);
                		}
                	}
                	if(isRefInPatterns.size() > 0)
                	{
                		isRef = true;
                		for(auto& patternTypeToReps : mPatternTypeToVoxelReps)
                		{
                			if(patternTypeToReps.first.second != t->type
                					&& H::contains(isRefInPatterns, patternTypeToReps.first.first))
                			{
                				for(uint32_t repTs : patternTypeToReps.second)
                				{
								    if(from <= repTs && repTs <= to && !H::contains(result,
								    		t->objects[repTs]))
								    {
								    	result.push_back(t->objects[repTs]);
								    }
                				}
                			}
                		}
                	}
                }
                if((!isRef && mFilterViaVgOut) || !mFilterViaVgOut)
                {
				    if(t->objects.size() > to)
				    {
					    {
							//TODO why did i do this?
						    result.insert(result.end(), t->objects.begin() + from, t->objects.begin() + to + 1);
					    }
				    } else if(t->objects.size() > from)
				    {
					    result.insert(result.end(), t->objects.begin() + from, t->objects.end());
				    }
                }
			}
		}
		/*
		ROS_INFO_STREAM("-------");
		for(auto& o : result)
		{
			if(o)
			{
				ROS_INFO_STREAM(o);
			}
		}
		ROS_INFO_STREAM("-------");
		*/
		return result;
    }

	std::vector<ISM::ObjectPtr> getOldObjects()
    {
		if(mTimestep == 0)
		{
			return std::vector<ISM::ObjectPtr>();
		}
		return getObjects(0, mTimestep -1);
    }

	void writeToDb()
	{
		uint32_t maxTimestep = std::get<1>(*mPoints.rbegin());
		if(maxTimestep > mTimestep)
		{
			applyAndGotoNextTimestep(maxTimestep - mTimestep);
			apply();
		}
        ROS_INFO_STREAM("Writing to database");
		apply();
        ISM::TracksPtr tracks(new ISM::Tracks(mTracks));
        auto objectSets = tracks->toObjectSetVector();
        for (auto objectSet : objectSets) {
            rec->insert(objectSet, mPatternName);
        }
        ROS_INFO_STREAM("Finished");
	}

	void writeToDb(const std_msgs::Empty)
	{
		writeToDb();
	}

private:
    int32_t mNextId;
    std::vector<int32_t> mIdXmlPoints;
    std::map<RecordGenPointPtr, std::vector<unsigned int> > mTimestampVisibleXmlPoints;
    dynamic_reconfigure::Server<asr_ism::recordGenConfig> mDynReconfServer;

public:
	unsigned int mTimestep;
	std::string mDbFile;
	bool mFilterViaVgOut;
	std::map<std::pair<std::string, std::string>, std::set<uint32_t>> mPatternTypeToVoxelReps;
	std::map<std::string, std::string> mPatternToRef;
	std::string mPatternName;

	std::set<std::tuple<uint32_t,uint32_t,RecordGenPointPtr>, cmpFromToPointTuple> mPoints;
	std::vector<ISM::TrackPtr> mTracks;
	ISM::RecorderPtr rec;
	bool mShowCurrent;
	bool mDoPublish;
	uint32_t mShowFrom;
	uint32_t mShowTo;
};

std_msgs::ColorRGBA getColorForTypeId(std::map<std::pair<std::string,std::string>, std_msgs::ColorRGBA >&
    typeIdToColor, std::vector<std_msgs::ColorRGBA>& allColors, uint32_t& availColorIndex,
    std::string& type, std::string& id)
{
	std_msgs::ColorRGBA result;
	if(typeIdToColor.find(std::make_pair(type,id)) != typeIdToColor.end())
	{
		result = typeIdToColor[std::make_pair(type,id)];
	} else
	{
		if(allColors.size() < availColorIndex + 1)
		{
			std::uniform_real_distribution<float> urd;
			std::default_random_engine re;
			result = VIZ::VizHelperRVIZ::createColorRGBA(urd(re), urd(re), urd(re), 1);
			allColors.push_back(result);
		} else
		{
			result = allColors[availColorIndex];
		}
		typeIdToColor[std::make_pair(type, id)] = result;
		++availColorIndex;
	}
	return result;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "recordGen");
	ros::NodeHandle n("~");
	ros::Rate r(1);
	std::string baseFrame = "/camera_left_frame";
	std::string visualizationTopic = "/visualization_marker";
	std::string dbFile;//("/media/share/data/own/gen1.sqlite");
    double bin_size = 0.03;
	double maxAngleDeviation = 10;
	bool useXml = false;
	bool useVgOut = false;
	boost::filesystem::path xmlFile;
	boost::filesystem::path vgOutFile;
    bool detailedBinNS;
	bool doPublish;

    if (!n.getParam("dbfilename", dbFile)) {
    	throw std::runtime_error("dbfilename not set");
    }
    ROS_INFO_STREAM("dbfilename: " << dbFile);

    std::string temp;
    if (n.getParam("xml", temp)) {
    	useXml = true;
    	xmlFile = temp;
    }
    xmlFile = temp;
    ROS_INFO_STREAM("xml: " << temp);

    if (n.getParam("vgOut", temp)) {
    	vgOutFile = temp;
    	if(boost::filesystem::is_regular_file(vgOutFile))
    	{
		    useVgOut = true;
    	} else
    	{
    		useVgOut = false;
    	}
    }
    vgOutFile = temp;
    if(useVgOut)
    {
	    ROS_INFO_STREAM("vgOut: " << vgOutFile);
    }

    if (!n.getParam("detailedBinNS", detailedBinNS)) {
        detailedBinNS = false;
    }
    ROS_INFO_STREAM("detailedBinNS: " << detailedBinNS);

    if (!n.getParam("doPublish", doPublish)) {
    	doPublish = false;
    }
    ROS_INFO_STREAM("doPublish: " << doPublish);

    if (!n.getParam("bin_size", bin_size)) {
        bin_size = 0.03;
    }
    ROS_INFO_STREAM("bin_size: " << bin_size);

    RecordGenerator rg(dbFile, "pattern0", doPublish);
    if(useXml)
    {
    	rg.parseXml(xmlFile);
    }
    if(useVgOut)
    {
    	rg.filterViaVgOut(vgOutFile);
    }

	ros::Publisher visPub = n.advertise<visualization_msgs::MarkerArray>(visualizationTopic, 100);


    ISM::VotingSpace vs = ISM::VotingSpace(bin_size, maxAngleDeviation);
    //std_msgs::ColorRGBA  regularBinColor = VizHelperRVIZ::createColorRGBA(0.0, 1.0, 0.0, 1.0);
    //std_msgs::ColorRGBA  regularObjectColor = VizHelperRVIZ::createColorRGBA(0.0, 1.0, 0.0, 1.0);
    //const double sphereRadius = 0.005;
    //const double markerLifetime = ros::Duration().toSec();

    /*
    ros::Subscriber s0 = n.subscribe<std_msgs::Float64MultiArray>("addPoint", 1000, &RecordGenerator::addPoint, &rg);
    ros::Subscriber s1 = n.subscribe<std_msgs::UInt32>("applyAndGotoNextTimestep2", 1000, &RecordGenerator::applyAndGotoNextTimestep, &rg);
    ros::Subscriber s3 = n.subscribe<std_msgs::Float64MultiArray>("changeParameter", 1000, &RecordGenerator::changeParameter, &rg);
    ros::Subscriber s4 = n.subscribe<std_msgs::String>("toogleInvis", 1000, &RecordGenerator::toogleInvisible, &rg);
    */
    ros::Subscriber s1 = n.subscribe<std_msgs::Float64MultiArray>("showFromTo", 1000, &RecordGenerator::showFromTo, &rg);
    //ros::Subscriber s1 = n.subscribe<std_msgs::UInt32MultiArray>("showFromTo", 1000, &RecordGenerator::showFromTo, &rg);
    ros::Subscriber s2 = n.subscribe<std_msgs::Empty>("writeToDb", 1000, &RecordGenerator::writeToDb, &rg);

    std::vector<std_msgs::ColorRGBA> allColors;
    allColors.push_back(VIZ::VizHelperRVIZ::createColorRGBA(0.0,0.0,0.0,1.0));
    allColors.push_back(VIZ::VizHelperRVIZ::createColorRGBA(1.0,0.0,0.0,1.0));
    allColors.push_back(VIZ::VizHelperRVIZ::createColorRGBA(0.0,1.0,0.0,1.0));
    allColors.push_back(VIZ::VizHelperRVIZ::createColorRGBA(0.0,0.0,1.0,1.0));
    allColors.push_back(VIZ::VizHelperRVIZ::createColorRGBA(0.5,0.2,0.9,1.0));
    allColors.push_back(VIZ::VizHelperRVIZ::createColorRGBA(0.3,0.8,0.8,1.0));
    allColors.push_back(VIZ::VizHelperRVIZ::createColorRGBA(0.3,0.5,0.5,1.0));
    allColors.push_back(VIZ::VizHelperRVIZ::createColorRGBA(0.5,0.3,0.1,1.0));

    std::map<std::pair<std::string,std::string>, std_msgs::ColorRGBA > typeIdToColor;
    //uint32_t availColorIndex = 0;

    /*
    boost::function< geometry_msgs::Point (int,int,int) > pointF = boost::bind(genCuboidPoint, _1, _2, _3, x, y, z, xwitdh, ywidth, zwidth);
std_msgs::ColorRGBA getColor(std::map<std::pair<std::string,std::string>, std_msgs::ColorRGBA >&
    typeIdToColor, std::vector<std_msgs::ColorRGBA>& allColors, uint32_t& availColorIndex,
    const std::string& type, const std::string& id)
    */

 /*   boost::function < std_msgs::ColorRGBA (const std::string,const std::string) > getColor =
                boost::bind(getColorForTypeId, boost::ref(typeIdToColor), boost::ref(allColors),
                                boost::ref(availColorIndex), _1, _2);
*/
    while (ros::ok()) {

        ///deleted for visualization refectoring. Bins now drawn at ism_voting_visualizer
    /*    std::map<std::tuple<int,int,int>,bool> binDrawn;

    	MarkerArray oBs;
    	MarkerArray bins;
        ros::spinOnce();
        //TODO
        if(rg.mShowCurrent)
        {
			for(ISM::ObjectPtr& o : rg.getCurrentVisibleObjects())
			{
                ISM::PointPtr point = o->pose->point;
				std::stringstream ss;
                ss << o->getType() << " With X:" << point->eigen.x() << "Y:" << point->eigen.y() << "Z:" << point->eigen.z();
				oBs.markers.push_back(VIZ::VizHelperRVIZ::createSphereMarker(point, baseFrame, ss.str(),
						boost::lexical_cast<int32_t>(o->getId()), sphereRadius, regularObjectColor, markerLifetime));
                //TODO: look for better solution than hard-code binSize; Problem introduced because of deletion of calcBinNum in VotingSpace
                int x = vs.discretizeToBins(point->eigen.x(), 0.1);
                int y = vs.discretizeToBins(point->eigen.y(), 0.1);
                int z = vs.discretizeToBins(point->eigen.z(), 0.1);
			    std::tuple<int,int,int> t = std::make_tuple(x, y, z);
			    if(binDrawn[t] == false)
			    {
				    std::stringstream ss;
				    ss << "\nObject" << o->getId() << "Is in Bin: "<< "X:" << x << " Y:" << y << " Z:" << z;
				    bins.markers.push_back(VIZ::VizHelperRVIZ::getBinMarker(x, y, z, bin_size, ss.str(), 0, baseFrame,
						    regularBinColor));
				    binDrawn[t] = true;
			    }
			}
			for(ISM::ObjectPtr& o : rg.getCurrentInvisibleObjects())
			{
                ISM::PointPtr point = o->pose->point;
				std::stringstream ss;
                ss << "Invis: " <<  o->getType() << " With X:" << point->eigen.x() << "Y:" << point->eigen.y() << "Z:" << point->eigen.z();
				visualization_msgs::Marker s = VizHelperRVIZ::createSphereMarker(point, baseFrame, ss.str(),
						boost::lexical_cast<int32_t>(o->getId()), sphereRadius, regularObjectColor, markerLifetime);
				s.color.g = 0.0f;
				s.color.b = 1.0f;
				oBs.markers.push_back(s);
                //TODO: look for better solution than hard-code binSize; Problem introduced because of deletion of calcBinNum in VotingSpace
                int x = vs.discretizeToBins(point->eigen.x(), 0.1);
                int y = vs.discretizeToBins(point->eigen.y(), 0.1);
                int z = vs.discretizeToBins(point->eigen.z(), 0.1);
			    std::tuple<int,int,int> t = std::make_tuple(x, y, z);
			    if(binDrawn[t] == false)
			    {
				    std::stringstream ss;
				    ss << "\nObject" << o->getId() << "Is in Bin: "<< "X:" << x << " Y:" << y << " Z:" << z;
				    bins.markers.push_back(VizHelperRVIZ::getBinMarker(x,y,z,bin_size,ss.str(), 0, baseFrame,
						    regularBinColor));
				    binDrawn[t] = true;
			    }
			}
			int id = 0;
			for(ISM::ObjectPtr& o : rg.getOldObjects())
			{
				if(!o)
					continue;
                ISM::PointPtr point = o->pose->point;
				std::stringstream ss;
                //ss << "Old " << o->getType() << " With X:" << point->eigen.x() << "Y:" << point->eigen.y() << "Z:" << point->eigen.z();
				ss << "Old " << o->getType();
				//visualization_msgs::Marker m = objectToSphere(o, baseFrame, ss.str(), boost::lexical_cast<int32_t>(o->getId()));
				visualization_msgs::Marker m = VizHelperRVIZ::createSphereMarker(point, baseFrame, ss.str(),
						boost::lexical_cast<int32_t>(id++), sphereRadius, regularObjectColor, markerLifetime);
				m.color.r = 1.0f;
				oBs.markers.push_back(m);
                //TODO: look for better solution than hard-code binSize; Problem introduced because of deletion of calcBinNum in VotingSpace
                int x = vs.discretizeToBins(point->eigen.x(), 0.1);
                int y = vs.discretizeToBins(point->eigen.y(), 0.1);
                int z = vs.discretizeToBins(point->eigen.z(), 0.1);
			    std::tuple<int,int,int> t = std::make_tuple(x, y, z);
			    if(binDrawn[t] == false)
			    {
				    std::stringstream ss;
				    ss << "\nObject" << o->getId() << "Is in Bin: "<< "X:" << x << " Y:" << y << " Z:" << z;
				    bins.markers.push_back(VizHelperRVIZ::getBinMarker(x,y,z,bin_size,ss.str(),0,baseFrame,
						    regularBinColor));
				    binDrawn[t] = true;
			    }
			}
        } else
        {
			int objectMarkerId = 0;
			int binMarkerId = 0;

			for(ISM::ObjectPtr& o : rg.getObjects(rg.mShowFrom, rg.mShowTo))
			{
				if(!rg.mDoPublish)
				{
					break;
				}
				if(!o)
				{
					continue;
				}
                ISM::PointPtr point = o->pose->point;
				std::stringstream ss;
                //ss << o->getType() << " With X:" << point->eigen.x() << "Y:" << point->eigen.y() << "Z:" << point->eigen.z();
				ss << o->getType() << " " << o->getId();
				auto sphereWithOrient = VIZ::VizHelperRVIZ::createSphereMarkerWithOrientation(
                        o->pose, baseFrame, ss.str(), objectMarkerId + 1, sphereRadius,
						getColor(o->getType(), o->getId()), markerLifetime, objectMarkerId + 2);
				objectMarkerId += 4;
				oBs.markers.insert(oBs.markers.end(), sphereWithOrient.markers.begin(),
						sphereWithOrient.markers.end());
                //TODO: look for better solution than hard-code binSize; Problem introduced because of deletion of calcBinNum in VotingSpace
                int x = vs.discretizeToBins(point->eigen.x(), 0.1);
                int y = vs.discretizeToBins(point->eigen.y(), 0.1);
                int z = vs.discretizeToBins(point->eigen.z(), 0.1);
			    std::tuple<int,int,int> t = std::make_tuple(x, y, z);
			    if(binDrawn[t] == false)
			    {
				    std::stringstream ss;
				    //ss << "\nObject" << o->getId() << "Is in Bin: "<< "X:" << x << " Y:" << y << " Z:" << z;
				    if(detailedBinNS)
				    {
                        ss << "Bin: "<< "X:" << x << " Y:" << y << " Z:" << z;
				    } else
				    {
                        ss << "Bins";
				    }
				    bins.markers.push_back(VIZ::VizHelperRVIZ::getBinMarker(x, y, z, bucketSize, ss.str(),
				    		binMarkerId++, baseFrame, regularBinColor));
				    binDrawn[t] = true;
			    }
			}

        }
        visPub.publish(oBs);
        visPub.publish(bins);*/

        r.sleep();
    }

}
