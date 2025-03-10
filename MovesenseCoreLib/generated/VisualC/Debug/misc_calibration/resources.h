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

struct WB_ALIGN(4) CalibrationSequence;

struct WB_ALIGN(4) CalibrationSequence
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 13824;

	WB_ALIGN(4) whiteboard::Array< whiteboard::FloatVector3D > vector;
};

namespace LOCAL 
{

struct ROOT;

struct MISC;

struct MISC_CALIBRATION;

struct MISC_CALIBRATION_MODE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 13824, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 13824;

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_INTERNAL_SERVER_ERROR> HTTP_CODE_INTERNAL_SERVER_ERROR;

		struct Parameters
		{
			struct MODE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint8 Type;
				typedef Type ConstReferenceType;
			};

			typedef MODE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Misc/Calibration/Mode */
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

			/** Gets MODE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::MODE::ConstReferenceType getMode() const
			{
				return mrParameterList[Parameters::MODE::Index].convertTo<Parameters::MODE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::MODE::ConstReferenceType)
		{
		}
	};
};

struct MISC_CALIBRATION_REFERENCES
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 13825, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 13825;

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_INTERNAL_SERVER_ERROR> HTTP_CODE_INTERNAL_SERVER_ERROR;

		struct Parameters
		{
			struct GRAVITY
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef float Type;
				typedef Type ConstReferenceType;
			};

			typedef GRAVITY Parameter1;

			struct MAGNETICFIELD
			{
				static const whiteboard::ParameterIndex Index = 1;

				typedef float Type;
				typedef Type ConstReferenceType;
			};

			typedef MAGNETICFIELD Parameter2;

			struct AVESAMPLESACCELERATION
			{
				static const whiteboard::ParameterIndex Index = 2;

				typedef uint8 Type;
				typedef Type ConstReferenceType;
			};

			typedef AVESAMPLESACCELERATION Parameter3;

			struct AVESAMPLESMAGNETICFIELD
			{
				static const whiteboard::ParameterIndex Index = 3;

				typedef uint8 Type;
				typedef Type ConstReferenceType;
			};

			typedef AVESAMPLESMAGNETICFIELD Parameter4;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 4;
		};

		/** Reference wrapper for strongly typed parameter list for /Misc/Calibration/References */
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

			/** Gets GRAVITY parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::GRAVITY::ConstReferenceType getGravity() const
			{
				return mrParameterList[Parameters::GRAVITY::Index].convertTo<Parameters::GRAVITY::ConstReferenceType>();
			}

			/** Gets MAGNETICFIELD parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::MAGNETICFIELD::ConstReferenceType getMagneticField() const
			{
				return mrParameterList[Parameters::MAGNETICFIELD::Index].convertTo<Parameters::MAGNETICFIELD::ConstReferenceType>();
			}

			/** Gets AVESAMPLESACCELERATION parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::AVESAMPLESACCELERATION::ConstReferenceType getAveSamplesAcceleration() const
			{
				return mrParameterList[Parameters::AVESAMPLESACCELERATION::Index].convertTo<Parameters::AVESAMPLESACCELERATION::ConstReferenceType>();
			}

			/** Gets AVESAMPLESMAGNETICFIELD parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::AVESAMPLESMAGNETICFIELD::ConstReferenceType getAveSamplesMagneticField() const
			{
				return mrParameterList[Parameters::AVESAMPLESMAGNETICFIELD::Index].convertTo<Parameters::AVESAMPLESMAGNETICFIELD::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::GRAVITY::ConstReferenceType,
			Parameters::MAGNETICFIELD::ConstReferenceType,
			Parameters::AVESAMPLESACCELERATION::ConstReferenceType,
			Parameters::AVESAMPLESMAGNETICFIELD::ConstReferenceType)
		{
		}
	};
};

struct MISC_CALIBRATION_SEQUENCE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 13826, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 13826;

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_INTERNAL_SERVER_ERROR> HTTP_CODE_INTERNAL_SERVER_ERROR;

		struct Parameters
		{
			struct SEQUENCE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef CalibrationSequence Type;
				typedef const Type& ConstReferenceType;
			};

			typedef SEQUENCE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Misc/Calibration/Sequence */
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

			/** Gets SEQUENCE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::SEQUENCE::ConstReferenceType getSequence() const
			{
				return mrParameterList[Parameters::SEQUENCE::Index].convertTo<Parameters::SEQUENCE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::SEQUENCE::ConstReferenceType)
		{
		}
	};
};

struct MISC_CALIBRATION_STATE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 13827, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 13827;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<uint8, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_INTERNAL_SERVER_ERROR> HTTP_CODE_INTERNAL_SERVER_ERROR;

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
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_INTERNAL_SERVER_ERROR> HTTP_CODE_INTERNAL_SERVER_ERROR;

		struct Parameters
		{
			struct STATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint8 Type;
				typedef Type ConstReferenceType;
			};

			typedef STATE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Misc/Calibration/State */
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

			/** Gets STATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::STATE::ConstReferenceType getState() const
			{
				return mrParameterList[Parameters::STATE::Index].convertTo<Parameters::STATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::STATE::ConstReferenceType)
		{
		}
	};
};



} // namespace LOCAL 

} // namespace WB_RES
