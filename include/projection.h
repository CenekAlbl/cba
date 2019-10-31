#pragma once
#include <vector>
#include <unordered_map>
#include <ceres/ceres.h>

namespace cba {

	class Projection
	{
	public:

		virtual ceres::CostFunction* getCostFunction(double const * const obsv, double weight, double const * const data = NULL) const = 0;
		virtual int getNumBlocks() const = 0;
		virtual int getResidualSize() const = 0;
		virtual const int * getBlockSizes() const = 0;
		virtual ~Projection(){};
	
	};

	class ProjFuncInterface {
	public:
		ProjFuncInterface(double const * const observations, double weight, double const * const data = NULL);
	
		double const * const obsv_;
		double const weight_;
		double const * const data_;
	};


	template<typename T> 
	Projection * createProjection() { return new T(); }

	class ProjectionFactory
	{
	public:

		static Projection * createProjectionInstance(std::string const projectionType);

	protected:
		typedef std::unordered_map<std::string, Projection*(*)()> MapType;

		static MapType & getMap() {
			// never deleted, exists until program termination 
			if (!map_) { map_ = new MapType; }
			return (*map_);
		}

	private:
		static MapType * map_;
	};

	template<class T>
	class ProjectionRegister : private ProjectionFactory
	{
	public:
		ProjectionRegister(std::string const & projectionType);
	};

	template<class T>
	ProjectionRegister<T>::ProjectionRegister(std::string const & projectionType) {
		getMap()[projectionType] = &createProjection<T>;
	}

	

}