//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** MiniaudioCpp **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/MiniaudioCpp
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2024 Jaroslav Pevno, MiniaudioCpp is offered under the terms of the ISC license:
//
//   Permission to use, copy, modify, and/or distribute this software for any purpose with or
//   without fee is hereby granted, provided that the above copyright notice and this permission
//   notice appear in all copies. THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
//   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
//   AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
//   CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
//   WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
//   CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#pragma once

// Thanks Jorrit Rouwe and his JoltPhysics library for the boilerplate below
// https://github.com/jrouwe/JoltPhysics

#include "Core.h"
#include <format>

namespace JPL
{
/// Trace function, needs to be overridden by application. This should output a line of text to the log / TTY.
using TraceFunction = void (*)(const char* message);
JPL_EXPORT extern TraceFunction Trace;

// Always turn on asserts in Debug mode
#if defined(_DEBUG) && !defined(JPL_ENABLE_ASSERTS)
#define JPL_ENABLE_ASSERTS
#ifndef JPL_TEST
#define JPL_ENABLE_ENSURE
#endif
#endif

#ifdef JPL_ENABLE_ASSERTS
/// Function called when an assertion fails. This function should return true if a breakpoint needs to be triggered
using AssertFailedFunction = bool(*)(const char* inExpression, const char* inMessage, const char* inFile, uint inLine);
JPL_EXPORT extern AssertFailedFunction AssertFailed;

// Helper functions to pass message on to failed function
struct AssertLastParam {};
inline bool AssertFailedParamHelper(const char* inExpression, const char* inFile, uint inLine, AssertLastParam) { return AssertFailed(inExpression, nullptr, inFile, inLine); }
inline bool AssertFailedParamHelper(const char* inExpression, const char* inFile, uint inLine, const char* inMessage, AssertLastParam) { return AssertFailed(inExpression, inMessage, inFile, inLine); }

/// Main assert macro, usage: JPL_ASSERT(condition, message) or JPL_ASSERT(condition)
#define JPL_ASSERT(inExpression, ...)	do { if (!(inExpression) && AssertFailedParamHelper(#inExpression, __FILE__, JPL::uint(__LINE__), ##__VA_ARGS__, JPL::AssertLastParam())) JPL_BREAKPOINT; } while (false)

#define JPL_IF_ENABLE_ASSERTS(...)		__VA_ARGS__
#else
#define JPL_ASSERT(...)					((void)0)

#define JPL_IF_ENABLE_ASSERTS(...)
#endif // JPL_ENABLE_ASSERTS


/// Define ENSURE
#ifndef JPL_ENSURE
#ifdef JPL_ENABLE_ENSURE

#define JPL_ENSURE(inExpression, ...) [&]{ if(!(inExpression)  && AssertFailedParamHelper(#inExpression, __FILE__, JPL::uint(__LINE__), ##__VA_ARGS__, JPL::AssertLastParam())) { JPL_BREAKPOINT; } return (inExpression); }()
#else
#define JPL_ENSURE(inExpression, ...) (inExpression)
#endif
#endif // !JPL_ENSURE

#ifndef JPL_TRACE_TAG
#define JPL_TRACE_TAG(tag, message) Trace(std::format("[{}]: Trace: {}", tag, message).c_str())
#endif // !JPL_TRACE_TAG

#ifndef JPL_INFO_TAG
#define JPL_INFO_TAG(tag, message) Trace(std::format("[{}]: Trace: {}", tag, message).c_str())
#endif // !JPL_INFO_TAG

#ifndef JPL_ERROR_TAG
#define JPL_ERROR_TAG(tag, message) Trace(std::format("[{}]: Trace: {}", tag, message).c_str())
#endif // !JPL_ERROR_TAG

} // namespace JPL
