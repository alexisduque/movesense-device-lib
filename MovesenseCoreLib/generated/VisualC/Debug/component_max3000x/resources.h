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

namespace LOCAL 
{

struct ROOT;

struct COMPONENT;

struct COMPONENT_MAX3000X;

struct COMPONENT_MAX3000X_HEARTRATEDATA
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2816, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2816;

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_BAD_REQUEST> HTTP_CODE_BAD_REQUEST;

		struct Parameters
		{
			struct ENABLED
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef bool Type;
				typedef Type ConstReferenceType;
			};

			typedef ENABLED Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Component/MAX3000x/HeartRateData */
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

			/** Gets ENABLED parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::ENABLED::ConstReferenceType getEnabled() const
			{
				return mrParameterList[Parameters::ENABLED::Index].convertTo<Parameters::ENABLED::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::ENABLED::ConstReferenceType)
		{
		}
	};

	struct SUBSCRIBE
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};

	struct EVENT
	{
		typedef HRData NotificationType;
		typedef const NotificationType& ConstReferenceNotificationType;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			const whiteboard::Api::OptionalParameter<ConstReferenceNotificationType>&)
		{
		}
	};

	struct UNSUBSCRIBE
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

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

struct COMPONENT_MAX3000X_REGISTER
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2817, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2817;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<uint32, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_INTERNAL_SERVER_ERROR> HTTP_CODE_INTERNAL_SERVER_ERROR;

		struct Parameters
		{
			struct ADDR
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint8 Type;
				typedef Type ConstReferenceType;
			};

			typedef ADDR Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Component/MAX3000x/Register */
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

			/** Gets ADDR parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::ADDR::ConstReferenceType getAddr() const
			{
				return mrParameterList[Parameters::ADDR::Index].convertTo<Parameters::ADDR::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::ADDR::ConstReferenceType)
		{
		}
	};

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_INTERNAL_SERVER_ERROR> HTTP_CODE_INTERNAL_SERVER_ERROR;

		struct Parameters
		{
			struct ADDR
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint8 Type;
				typedef Type ConstReferenceType;
			};

			typedef ADDR Parameter1;

			struct VALUE
			{
				static const whiteboard::ParameterIndex Index = 1;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			typedef VALUE Parameter2;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 2;
		};

		/** Reference wrapper for strongly typed parameter list for /Component/MAX3000x/Register */
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

			/** Gets ADDR parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::ADDR::ConstReferenceType getAddr() const
			{
				return mrParameterList[Parameters::ADDR::Index].convertTo<Parameters::ADDR::ConstReferenceType>();
			}

			/** Gets VALUE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::VALUE::ConstReferenceType getValue() const
			{
				return mrParameterList[Parameters::VALUE::Index].convertTo<Parameters::VALUE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::ADDR::ConstReferenceType,
			Parameters::VALUE::ConstReferenceType)
		{
		}
	};
};

struct COMPONENT_MAX3000X_VERSION
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2818, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2818;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const VersionInfoArray&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
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
};

struct COMPONENT_MAX3000X_WAKEUP
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2819, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2819;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<uint8, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

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
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_BAD_REQUEST> HTTP_CODE_BAD_REQUEST;

		struct Parameters
		{
			struct NEWSTATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint8 Type;
				typedef Type ConstReferenceType;
			};

			typedef NEWSTATE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Component/MAX3000x/WakeUp */
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

			/** Gets NEWSTATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::NEWSTATE::ConstReferenceType getNewState() const
			{
				return mrParameterList[Parameters::NEWSTATE::Index].convertTo<Parameters::NEWSTATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::NEWSTATE::ConstReferenceType)
		{
		}
	};
};



} // namespace LOCAL 

} // namespace WB_RES
