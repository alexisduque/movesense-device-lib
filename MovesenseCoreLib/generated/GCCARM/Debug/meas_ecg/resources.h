#pragma once
/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/
// Copyright (c) Suunto Oy 2014 - 2017. All rights reserved.

#include "whiteboard/Identifiers.h"
#include "whiteboard/ParameterList.h"
#include "whiteboard/Result.h"
#include "whiteboard/ResourceClient.h"

#include "whiteboard/builtinTypes/Array.h"
#include "whiteboard/builtinTypes/ByteStream.h"
#include "whiteboard/builtinTypes/Date.h"
#include "whiteboard/builtinTypes/DateTime.h"
#include "whiteboard/builtinTypes/Optional.h"
#include "whiteboard/builtinTypes/Structures.h"
#include "whiteboard/builtinTypes/Time.h"
#include "whiteboard/builtinTypes/Timestamp.h"
#include "whiteboard/builtinTypes/TypedEnum.h"
#include "whiteboard/builtinTypes/Vector2D.h"
#include "whiteboard/builtinTypes/Vector3D.h"
#include "whiteboard/builtinTypes/WrapperFor32BitPointer.h"

#define WB_RESOURCE_VALUE(whiteboardId, localResourceId, executionContextId) \
    static_cast<whiteboard::ResourceId::Value>( \
        (static_cast<uint32>(localResourceId) << 16) | \
        (static_cast<uint32>(whiteboardId) << 8) | \
        (static_cast<uint32>(executionContextId) << 4) | \
        (static_cast<uint32>(whiteboard::ID_INVALID_RESOURCE_INSTANCE)))


#include "../wb-resources/resources.h"
#include "../movesense_types/resources.h"

#define WB_EXEC_CTX_APPLICATION                  static_cast<whiteboard::ExecutionContextId>(0)
#define WB_EXEC_CTX_MEAS                         static_cast<whiteboard::ExecutionContextId>(1)

namespace WB_RES {

struct WB_ALIGN(4) ECGInfo;
struct WB_ALIGN(4) ECGConfig;
struct WB_ALIGN(4) ECGData;
struct WB_ALIGN(4) ECGDataVoltage;

struct WB_ALIGN(4) ECGInfo
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 17152;

	WB_ALIGN(2) whiteboard::Optional< uint16 > currentSampleRate;
	WB_ALIGN(4) whiteboard::Array< uint16 > availableSampleRates;
	WB_ALIGN(2) uint16 arraySize;
	WB_ALIGN(4) whiteboard::Array< uint16 > lowPass;
	WB_ALIGN(4) whiteboard::Array< float > highPass;
};

struct WB_ALIGN(4) ECGConfig
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 17153;

	WB_ALIGN(2) whiteboard::Optional< uint16 > lowPass;
	WB_ALIGN(4) whiteboard::Optional< float > highPass;
};

struct WB_ALIGN(4) ECGData
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 17154;

	WB_ALIGN(4) whiteboard::Array< int32 > samples;
	WB_ALIGN(4) uint32 timestamp;
};

struct WB_ALIGN(4) ECGDataVoltage
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 17155;

	WB_ALIGN(4) whiteboard::Array< float > samples;
	WB_ALIGN(4) uint32 timestamp;
};

namespace LOCAL 
{

struct ROOT;

struct MEAS;

struct MEAS_ECG;

struct MEAS_ECG_CONFIG
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 17152, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 17152;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const ECGConfig&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_SERVICE_UNAVAILABLE> HTTP_CODE_SERVICE_UNAVAILABLE;

		struct Parameters
		{
			struct CONFIG
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef ECGConfig Type;
				typedef const Type& ConstReferenceType;
			};

			typedef CONFIG Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Meas/ECG/Config */
		class ParameterListRef
		{
		private:
			/** Prevent use of default constructor */
			ParameterListRef() DELETED;

			/** Prevent use of copy constructor */
			ParameterListRef(const ParameterListRef&) DELETED;

			/** Prevent use of assignment operator */
			const ParameterListRef& operator=(const ParameterListRef&) DELETED;

		public:
			/** Constructor that initializes this class from existing parameter list
			*
			* @param rParameterList Reference to parameter list that contains untyped parameters
			*/
			inline ParameterListRef(const whiteboard::ParameterList& rParameterList)
				: mrParameterList(rParameterList)
			{
			}

			/** Gets CONFIG parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::CONFIG::ConstReferenceType getConfig() const
			{
				return mrParameterList[Parameters::CONFIG::Index].convertTo<Parameters::CONFIG::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::CONFIG::ConstReferenceType)
		{
		}
	};
};

struct MEAS_ECG_INFO
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 17153, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 17153;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const ECGInfo&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};
};

struct MEAS_ECG_REQUIREDSAMPLERATE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 17154, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 17154;

	struct SUBSCRIBE
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_NOT_IMPLEMENTED> HTTP_CODE_NOT_IMPLEMENTED;

		struct Parameters
		{
			struct REQUIREDSAMPLERATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			typedef REQUIREDSAMPLERATE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Meas/ECG/{RequiredSampleRate} */
		class ParameterListRef
		{
		private:
			/** Prevent use of default constructor */
			ParameterListRef() DELETED;

			/** Prevent use of copy constructor */
			ParameterListRef(const ParameterListRef&) DELETED;

			/** Prevent use of assignment operator */
			const ParameterListRef& operator=(const ParameterListRef&) DELETED;

		public:
			/** Constructor that initializes this class from existing parameter list
			*
			* @param rParameterList Reference to parameter list that contains untyped parameters
			*/
			inline ParameterListRef(const whiteboard::ParameterList& rParameterList)
				: mrParameterList(rParameterList)
			{
			}

			/** Gets REQUIREDSAMPLERATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::REQUIREDSAMPLERATE::ConstReferenceType getRequiredSampleRate() const
			{
				return mrParameterList[Parameters::REQUIREDSAMPLERATE::Index].convertTo<Parameters::REQUIREDSAMPLERATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::REQUIREDSAMPLERATE::ConstReferenceType)
		{
		}
	};

	struct EVENT
	{
		typedef ECGData NotificationType;
		typedef const NotificationType& ConstReferenceNotificationType;

		struct Parameters
		{
			struct REQUIREDSAMPLERATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			typedef REQUIREDSAMPLERATE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Meas/ECG/{RequiredSampleRate} */
		class ParameterListRef
		{
		private:
			/** Prevent use of default constructor */
			ParameterListRef() DELETED;

			/** Prevent use of copy constructor */
			ParameterListRef(const ParameterListRef&) DELETED;

			/** Prevent use of assignment operator */
			const ParameterListRef& operator=(const ParameterListRef&) DELETED;

		public:
			/** Constructor that initializes this class from existing parameter list
			*
			* @param rParameterList Reference to parameter list that contains untyped parameters
			*/
			inline ParameterListRef(const whiteboard::ParameterList& rParameterList)
				: mrParameterList(rParameterList)
			{
			}

			/** Checks whether optional parameter REQUIREDSAMPLERATE has a value
			*
			* @return A value indicating whether the parameter has a value
			*/
			inline bool hasRequiredSampleRate() const
			{
				if (mrParameterList.getNumberOfParameters() <= Parameters::REQUIREDSAMPLERATE::Index)
				{
					return false;
				}

				return mrParameterList[Parameters::REQUIREDSAMPLERATE::Index].getType() != whiteboard::WB_TYPE_NONE;
			}

			/** Gets REQUIREDSAMPLERATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::REQUIREDSAMPLERATE::ConstReferenceType getRequiredSampleRate() const
			{
				return mrParameterList[Parameters::REQUIREDSAMPLERATE::Index].convertTo<Parameters::REQUIREDSAMPLERATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			const whiteboard::Api::OptionalParameter<ConstReferenceNotificationType>&,
			const whiteboard::Api::OptionalParameter<Parameters::REQUIREDSAMPLERATE::ConstReferenceType>& = whiteboard::NoType::NoValue)
		{
		}
	};

	struct UNSUBSCRIBE
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			struct REQUIREDSAMPLERATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			typedef REQUIREDSAMPLERATE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Meas/ECG/{RequiredSampleRate} */
		class ParameterListRef
		{
		private:
			/** Prevent use of default constructor */
			ParameterListRef() DELETED;

			/** Prevent use of copy constructor */
			ParameterListRef(const ParameterListRef&) DELETED;

			/** Prevent use of assignment operator */
			const ParameterListRef& operator=(const ParameterListRef&) DELETED;

		public:
			/** Constructor that initializes this class from existing parameter list
			*
			* @param rParameterList Reference to parameter list that contains untyped parameters
			*/
			inline ParameterListRef(const whiteboard::ParameterList& rParameterList)
				: mrParameterList(rParameterList)
			{
			}

			/** Gets REQUIREDSAMPLERATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::REQUIREDSAMPLERATE::ConstReferenceType getRequiredSampleRate() const
			{
				return mrParameterList[Parameters::REQUIREDSAMPLERATE::Index].convertTo<Parameters::REQUIREDSAMPLERATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::REQUIREDSAMPLERATE::ConstReferenceType)
		{
		}
	};
};

struct MEAS_ECG_REQUIREDSAMPLERATE_MV
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 17155, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 17155;

	struct SUBSCRIBE
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_NOT_IMPLEMENTED> HTTP_CODE_NOT_IMPLEMENTED;

		struct Parameters
		{
			struct REQUIREDSAMPLERATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			typedef REQUIREDSAMPLERATE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Meas/ECG/{RequiredSampleRate}/mV */
		class ParameterListRef
		{
		private:
			/** Prevent use of default constructor */
			ParameterListRef() DELETED;

			/** Prevent use of copy constructor */
			ParameterListRef(const ParameterListRef&) DELETED;

			/** Prevent use of assignment operator */
			const ParameterListRef& operator=(const ParameterListRef&) DELETED;

		public:
			/** Constructor that initializes this class from existing parameter list
			*
			* @param rParameterList Reference to parameter list that contains untyped parameters
			*/
			inline ParameterListRef(const whiteboard::ParameterList& rParameterList)
				: mrParameterList(rParameterList)
			{
			}

			/** Gets REQUIREDSAMPLERATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::REQUIREDSAMPLERATE::ConstReferenceType getRequiredSampleRate() const
			{
				return mrParameterList[Parameters::REQUIREDSAMPLERATE::Index].convertTo<Parameters::REQUIREDSAMPLERATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::REQUIREDSAMPLERATE::ConstReferenceType)
		{
		}
	};

	struct EVENT
	{
		typedef ECGDataVoltage NotificationType;
		typedef const NotificationType& ConstReferenceNotificationType;

		struct Parameters
		{
			struct REQUIREDSAMPLERATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			typedef REQUIREDSAMPLERATE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Meas/ECG/{RequiredSampleRate}/mV */
		class ParameterListRef
		{
		private:
			/** Prevent use of default constructor */
			ParameterListRef() DELETED;

			/** Prevent use of copy constructor */
			ParameterListRef(const ParameterListRef&) DELETED;

			/** Prevent use of assignment operator */
			const ParameterListRef& operator=(const ParameterListRef&) DELETED;

		public:
			/** Constructor that initializes this class from existing parameter list
			*
			* @param rParameterList Reference to parameter list that contains untyped parameters
			*/
			inline ParameterListRef(const whiteboard::ParameterList& rParameterList)
				: mrParameterList(rParameterList)
			{
			}

			/** Checks whether optional parameter REQUIREDSAMPLERATE has a value
			*
			* @return A value indicating whether the parameter has a value
			*/
			inline bool hasRequiredSampleRate() const
			{
				if (mrParameterList.getNumberOfParameters() <= Parameters::REQUIREDSAMPLERATE::Index)
				{
					return false;
				}

				return mrParameterList[Parameters::REQUIREDSAMPLERATE::Index].getType() != whiteboard::WB_TYPE_NONE;
			}

			/** Gets REQUIREDSAMPLERATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::REQUIREDSAMPLERATE::ConstReferenceType getRequiredSampleRate() const
			{
				return mrParameterList[Parameters::REQUIREDSAMPLERATE::Index].convertTo<Parameters::REQUIREDSAMPLERATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			const whiteboard::Api::OptionalParameter<ConstReferenceNotificationType>&,
			const whiteboard::Api::OptionalParameter<Parameters::REQUIREDSAMPLERATE::ConstReferenceType>& = whiteboard::NoType::NoValue)
		{
		}
	};

	struct UNSUBSCRIBE
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			struct REQUIREDSAMPLERATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			typedef REQUIREDSAMPLERATE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Meas/ECG/{RequiredSampleRate}/mV */
		class ParameterListRef
		{
		private:
			/** Prevent use of default constructor */
			ParameterListRef() DELETED;

			/** Prevent use of copy constructor */
			ParameterListRef(const ParameterListRef&) DELETED;

			/** Prevent use of assignment operator */
			const ParameterListRef& operator=(const ParameterListRef&) DELETED;

		public:
			/** Constructor that initializes this class from existing parameter list
			*
			* @param rParameterList Reference to parameter list that contains untyped parameters
			*/
			inline ParameterListRef(const whiteboard::ParameterList& rParameterList)
				: mrParameterList(rParameterList)
			{
			}

			/** Gets REQUIREDSAMPLERATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::REQUIREDSAMPLERATE::ConstReferenceType getRequiredSampleRate() const
			{
				return mrParameterList[Parameters::REQUIREDSAMPLERATE::Index].convertTo<Parameters::REQUIREDSAMPLERATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::REQUIREDSAMPLERATE::ConstReferenceType)
		{
		}
	};
};



} // namespace LOCAL 

} // namespace WB_RES
