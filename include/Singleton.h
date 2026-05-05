#ifndef VRPSTW_SINGLETON_H
#define VRPSTW_SINGLETON_H

#include <stdlib.h>

namespace vrpstw
{
	template<typename T>
	class Singleton
	{
	public:
		virtual ~Singleton() {}

		static T& getInstance()
		{
			if (singleton == NULL)
				singleton = new T();
			return *singleton;
		}

		static inline const T& getBuilt()
		{
			return *singleton;
		}

		void destroy()
		{
			delete singleton;
		}

	
		Singleton() {}
		static T* singleton;
	};

	template<typename T>
	T* Singleton<T>::singleton = NULL;
}

#endif
