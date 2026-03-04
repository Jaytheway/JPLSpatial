//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
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

#include <atomic>
#include <cassert>
#include <memory_resource>
#include <type_traits>

namespace JPL
{
//==============================================================================
enum class ThreadType
{
    realtime,
    nonRealtime
};

enum class RealtimeObjectOptions
{
    nonRealtimeMutatable,
    realtimeMutatable
};

namespace detail
{
    // Forward declaration
    template<typename T>
    class NonRealtimeMutatable;
    
    template<typename T>
    class RealtimeMutatable;
}

// By default we remove lock in non-realtime access.
// The object assumed to be only used by a single non-realtime thread.
#ifndef JPL_RT_OBJECT_NON_REALTIME_ACCESS_LOCK
#define JPL_RT_OBJECT_NON_REALTIME_ACCESS_LOCK 0
#endif

//==============================================================================
/// Useful class to synchronise access to an object from multiple threads with
/// the additional feature that one designated thread will never wait to get
/// access to the object.
/// 
/// Essentially Faiban Renn's RealtimeObject (https://github.com/hogliux/farbot),
/// with some modifications:
/// - memory is managed by polymorphic allocator instead of std::unique_ptr
/// - asumed only one non-realtime thread will access the object
template <typename T, RealtimeObjectOptions Options>
class RealtimeObject
{
public:
    using allocator_type = std::pmr::polymorphic_allocator<>;

public:
    /** Creates a default constructed T */
    RealtimeObject() = default;

    /** Creates a copy of T */
    explicit RealtimeObject(const T& obj, const allocator_type& allocator = {}) : mImpl(obj, allocator) {}

    /** Moves T into this realtime wrapper */
    explicit RealtimeObject(T&& obj, const allocator_type& allocator = {}) : mImpl(std::move(obj), allocator) {}

    ~RealtimeObject() = default;

    //==============================================================================
    using RealtimeAcquireReturnType = std::conditional_t<Options == RealtimeObjectOptions::nonRealtimeMutatable, const T, T>;
    using NonRealtimeAcquireReturnType = std::conditional_t<Options == RealtimeObjectOptions::realtimeMutatable, const T, T>;

    //==============================================================================
    /** Returns a reference to T. Use this method on the real-time thread.
     *  It must be matched by realtimeRelease when you are finished using the
     *  object. Alternatively, use the ScopedAccess helper class below.
     *
     *  Only a single real-time thread can acquire this object at once!
     *
     *  This method is wait- and lock-free.
     */
    RealtimeAcquireReturnType& realtimeAcquire() noexcept { return mImpl.realtimeAcquire(); }

    /** Releases the lock on T previously acquired by realtimeAcquire.
     *
     *  This method is wait- and lock-free.
     */
    void realtimeRelease() noexcept { mImpl.realtimeRelease(); }

	/** Replace the underlying value with a new instance of T by forwarding
	*  the method's arguments to T's constructor
	*/
    template <typename... Args>
    void realtimeReplace(Args&& ... args) noexcept requires (Options == RealtimeObjectOptions::realtimeMutatable)
    {
        mImpl.realtimeReplace(std::forward<Args>(args)...);
    }

    //==============================================================================
    /** Returns a reference to T. Use this method on the non real-time thread.
     *  It must be matched by nonRealtimeRelease when you are finished using
     *  the object. Alternatively, use the ScopedAccess helper class below.
     *
     *  Multiple non-realtime threads can acquire an object at the same time.
     *
     *  This method uses a lock should not be used on a realtime thread.
     */
    NonRealtimeAcquireReturnType& nonRealtimeAcquire() { return mImpl.nonRealtimeAcquire(); }

    /** Releases the lock on T previously acquired by nonRealtimeAcquire.
     *
     *  This method uses both a lock and a spin loop and should not be used
     *  on a realtime thread.
     */
    void nonRealtimeRelease() { mImpl.nonRealtimeRelease(); }

    /** Replace the underlying value with a new instance of T by forwarding
     *  the method's arguments to T's constructor
     */
    template <typename... Args>
    void nonRealtimeReplace(Args&& ... args) requires(Options == RealtimeObjectOptions::nonRealtimeMutatable)
    {
        mImpl.nonRealtimeReplace(std::forward<Args>(args)...);
    }

    //==============================================================================
    /** Instead of calling acquire and release manually, you can also use this RAII
     *  version which calls acquire automatically on construction and release when
     *  destructed.
     */
    template <ThreadType threadType>
    class ScopedAccess : public std::conditional_t<Options == RealtimeObjectOptions::realtimeMutatable,
                                                    detail::RealtimeMutatable<T>,
                                                    detail::NonRealtimeMutatable<T>>
                                    ::template ScopedAccess<threadType == ThreadType::realtime>
    {
    public:
        explicit ScopedAccess(RealtimeObject& parent)
            : Impl::template ScopedAccess<threadType == ThreadType::realtime>(parent.mImpl) {}

#if defined(DOXYGEN) and DOXYGEN
        // Various ways to get access to the underlying object.
        // Non-const method are only supported on the realtime
        // or non-realtime thread as indicated by the Options
        // template argument
        T* get() noexcept;
        const T* get() const noexcept;
        T& operator *() noexcept;
        const T& operator *() const noexcept;
        T* operator->() noexcept;
        const T* operator->() const noexcept;
#endif

        //==============================================================================
        ScopedAccess(const ScopedAccess&) = delete;
        ScopedAccess(ScopedAccess&&) = delete;
        ScopedAccess& operator=(const ScopedAccess&) = delete;
        ScopedAccess& operator=(ScopedAccess&&) = delete;
    };
private:
    using Impl = std::conditional_t<Options == RealtimeObjectOptions::realtimeMutatable,
                    detail::RealtimeMutatable<T>,
                    detail::NonRealtimeMutatable<T>>;
    Impl mImpl;
};

} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
    namespace detail
    {
        //======================================================================
        // Forward declaration
        template <typename, bool>
        class NRMScopedAccessImpl;

        template<typename T>
        class NonRealtimeMutatable
        {
        public:
            using allocator_type = std::pmr::polymorphic_allocator<>;

        public:
            NonRealtimeMutatable(const allocator_type& allocator = {})
                : mAllocator(allocator)
                , storage(mAllocator.new_object<T>())
                , pointer(storage)
            {
            }

            NonRealtimeMutatable(const T& obj, const allocator_type& allocator = {})
                : mAllocator(allocator)
                , storage(mAllocator.new_object<T>(obj))
                , pointer(storage)
            {
            }

            NonRealtimeMutatable(T&& obj, const allocator_type& allocator = {})
                : mAllocator(allocator)
                , storage(mAllocator.new_object<T>(std::move(obj)))
                , pointer(storage)
            {
            }

            ~NonRealtimeMutatable()
            {
                assert(pointer.load() != nullptr); // <- never delete this object while the realtime thread is holding the lock

                // Spin util realtime-thread releases the object!
                while (pointer.load() == nullptr);

                if (storage)
                    mAllocator.delete_object(storage);
                if (copy)
                    mAllocator.delete_object(copy);

#if JPL_RT_OBJECT_NON_REALTIME_ACCESS_LOCK
                auto accquired = nonRealtimeLock.try_lock();

                ((void)(accquired));
                assert(accquired);  // <- you didn't call release on one of the non-realtime threads before deleting this object

                nonRealtimeLock.unlock();
#endif
            }

            template <typename... Args>
            static NonRealtimeMutatable create(Args&& ... args)
            {
                return RealtimeObject(T(std::forward<Args>(args)...));
            }

            const T& realtimeAcquire() noexcept
            {
                assert(pointer.load() != nullptr); // <- You didn't balance your acquire and release calls!
                currentObj = pointer.exchange(nullptr);
                return *currentObj;
            }

            void realtimeRelease() noexcept
            {
                // You didn't balance your acquire and release calls
                assert(pointer.load() == nullptr);

                // replace back
                pointer.store(currentObj);
            }

            T& nonRealtimeAcquire()
            {
#if JPL_RT_OBJECT_NON_REALTIME_ACCESS_LOCK
                nonRealtimeLock.lock();
#endif
                if (copy)
                    mAllocator.delete_object(copy);
                // TODO: should we just destruct -> in-place construct?
                copy = mAllocator.new_object<T>(*storage);

                return *copy;
            }

            void nonRealtimeRelease()
            {
                T* ptr;

                // block until realtime thread is done using the object
                do
                {
                    ptr = storage;
                } while (!pointer.compare_exchange_weak(ptr, copy));
                
                // transfrer copy to storage
                mAllocator.delete_object(storage);
                storage = copy;
                copy = nullptr;

#if JPL_RT_OBJECT_NON_REALTIME_ACCESS_LOCK
                nonRealtimeLock.unlock();
#endif
            }

            template <typename... Args>
            void nonRealtimeReplace(Args && ... args)
            {
#if JPL_RT_OBJECT_NON_REALTIME_ACCESS_LOCK
                nonRealtimeLock.lock();
#endif
                if (copy)
                    mAllocator.delete_object(copy);
                copy = mAllocator.new_object<T>(std::forward<Args>(args)...);

                nonRealtimeRelease();
            }

            template <bool isRealtimeThread>
            class ScopedAccess : public NRMScopedAccessImpl<T, isRealtimeThread>
            {
            public:
                explicit ScopedAccess(NonRealtimeMutatable& parent)
                    : NRMScopedAccessImpl<T, isRealtimeThread>(parent) {}
                ScopedAccess(const ScopedAccess&) = delete;
                ScopedAccess(ScopedAccess&&) = delete;
                ScopedAccess& operator=(const ScopedAccess&) = delete;
                ScopedAccess& operator=(ScopedAccess&&) = delete;
            };
        private:
            friend class NRMScopedAccessImpl<T, true>;
            friend class NRMScopedAccessImpl<T, false>;
            //explicit NonRealtimeMutatable(std::unique_ptr<T>&& u);

            allocator_type mAllocator;

            T* storage = nullptr;
            std::atomic<T*> pointer;

#if JPL_RT_OBJECT_NON_REALTIME_ACCESS_LOCK
            std::mutex nonRealtimeLock;
#endif
            T* copy = nullptr;

            // only accessed by realtime thread
            T* currentObj = nullptr;
        };

        template <typename T, bool>
        class NRMScopedAccessImpl
        {
        protected:
            NRMScopedAccessImpl(NonRealtimeMutatable<T>& parent)
                : p(parent), currentValue(&p.nonRealtimeAcquire())
            {
            }
            ~NRMScopedAccessImpl() { p.nonRealtimeRelease(); }
        public:
            T* get() noexcept { return currentValue; }
            const T* get() const noexcept { return currentValue; }
            T& operator *() noexcept { return *currentValue; }
            const T& operator *() const noexcept { return *currentValue; }
            T* operator->() noexcept { return currentValue; }
            const T* operator->() const noexcept { return currentValue; }
        private:
            NonRealtimeMutatable<T>& p;
            T* currentValue;
        };

        template <typename T>
        class NRMScopedAccessImpl<T, true>
        {
        protected:
            NRMScopedAccessImpl(NonRealtimeMutatable<T>& parent) noexcept
                : p(parent), currentValue(&p.realtimeAcquire())
            {
            }
            ~NRMScopedAccessImpl() noexcept { p.realtimeRelease(); }
        public:
            const T* get() const noexcept { return currentValue; }
            const T& operator *() const noexcept { return *currentValue; }
            const T* operator->() const noexcept { return currentValue; }
        private:
            NonRealtimeMutatable<T>& p;
            const T* currentValue;
        };

        //======================================================================
        template <typename, bool> class RMScopedAccessImpl;
        template <typename T> class RealtimeMutatable
        {
        public:
            RealtimeMutatable() = default;

            explicit RealtimeMutatable(const T& obj) : data({ obj, obj }), realtimeCopy(obj) {}

            ~RealtimeMutatable()
            {
                assert((control.load() & BUSY_BIT) == 0); // <- never delete this object while the realtime thread is still using it

                // Spin!
                while ((control.load() & BUSY_BIT) == 1);

#if JPL_RT_OBJECT_NON_REALTIME_ACCESS_LOCK
                auto accquired = nonRealtimeLock.try_lock();

                ((void)(accquired));
                assert(accquired);  // <- you didn't call release on one of the non-realtime threads before deleting this object

                nonRealtimeLock.unlock();
#endif
            }

            template <typename... Args>
            static RealtimeMutatable create(Args && ... args)
            {
                return RealtimeObject(false, std::forward<Args>(args)...);
            }

            T& realtimeAcquire() noexcept
            {
                return realtimeCopy;
            }

            void realtimeRelease() noexcept
            {
                auto idx = acquireIndex();
                data[idx] = realtimeCopy;
                releaseIndex(idx);
            }

            template <typename... Args>
            void realtimeReplace(Args && ... args)
            {
                T obj(std::forward<Args>(args)...);

                auto idx = acquireIndex();
                data[idx] = std::move(obj);
                releaseIndex(idx);
            }

            const T& nonRealtimeAcquire()
            {
#if JPL_RT_OBJECT_NON_REALTIME_ACCESS_LOCK
                nonRealtimeLock.lock();
#endif
                auto current = control.load(std::memory_order_acquire);

                // there is new data so flip the indices around atomically ensuring we are not inside realtimeAssign
                if ((current & NEWDATA_BIT) != 0)
                {
                    int newValue;

                    do
                    {
                        // expect the realtime thread not to be inside the realtime-assign
                        current &= ~BUSY_BIT;

                        // realtime thread should flip index value and clear the newdata bit
                        newValue = (current ^ INDEX_BIT) & INDEX_BIT;
                    } while (!control.compare_exchange_weak(current, newValue, std::memory_order_acq_rel));

                    current = newValue;
                }

                // flip the index bit as we always use the index that the realtime thread is currently NOT using
                auto nonRealtimeIndex = (current & INDEX_BIT) ^ 1;

                return data[nonRealtimeIndex];
            }

            void nonRealtimeRelease()
            {
#if JPL_RT_OBJECT_NON_REALTIME_ACCESS_LOCK
                nonRealtimeLock.unlock();
#endif
            }

            template <bool isRealtimeThread>
            class ScopedAccess : public RMScopedAccessImpl<T, isRealtimeThread>
            {
            public:
                explicit ScopedAccess(RealtimeMutatable& parent) : RMScopedAccessImpl<T, isRealtimeThread>(parent) {}
                ScopedAccess(const ScopedAccess&) = delete;
                ScopedAccess(ScopedAccess&&) = delete;
                ScopedAccess& operator=(const ScopedAccess&) = delete;
                ScopedAccess& operator=(ScopedAccess&&) = delete;
            };
        private:
            friend class RMScopedAccessImpl<T, false>;
            friend class RMScopedAccessImpl<T, true>;

            template <typename... Args>
            explicit RealtimeMutatable(bool, Args &&... args)
                : data({ T(std::forward(args)...), T(std::forward(args)...) }), realtimeCopy(std::forward(args)...)
            {
            }

            enum
            {
                INDEX_BIT = (1 << 0),
                BUSY_BIT = (1 << 1),
                NEWDATA_BIT = (1 << 2)
            };

            int acquireIndex() noexcept
            {
                return control.fetch_or(BUSY_BIT, std::memory_order_acquire) & INDEX_BIT;
            }

            void releaseIndex(int idx) noexcept
            {
                control.store((idx & INDEX_BIT) | NEWDATA_BIT, std::memory_order_release);
            }

            std::atomic<int> control = { 0 };

            std::array<T, 2> data;
            T realtimeCopy;
#if JPL_RT_OBJECT_NON_REALTIME_ACCESS_LOCK
            std::mutex nonRealtimeLock;
#endif
        };

        template <typename T, bool>
        class RMScopedAccessImpl
        {
        protected:
            RMScopedAccessImpl(RealtimeMutatable<T>& parent)
                : p(parent),
                currentValue(&p.realtimeAcquire())
            {
            }

            ~RMScopedAccessImpl()
            {
                p.realtimeRelease();
            }
        public:
            T* get() noexcept { return currentValue; }
            const T* get() const noexcept { return currentValue; }
            T& operator *() noexcept { return *currentValue; }
            const T& operator *() const noexcept { return *currentValue; }
            T* operator->() noexcept { return currentValue; }
            const T* operator->() const noexcept { return currentValue; }
        private:
            RealtimeMutatable<T>& p;
            T* currentValue;
        };

        template <typename T>
        class RMScopedAccessImpl<T, false>
        {
        protected:
            RMScopedAccessImpl(RealtimeMutatable<T>& parent) noexcept
                : p(parent), currentValue(&p.nonRealtimeAcquire())
            {
            }
            ~RMScopedAccessImpl() noexcept { p.nonRealtimeRelease(); }
        public:
            const T* get() const noexcept { return currentValue; }
            const T& operator *() const noexcept { return *currentValue; }
            const T* operator->() const noexcept { return currentValue; }
        private:
            RealtimeMutatable<T>& p;
            const T* currentValue;
        };
    } // namespace detail
} // namespace JPL
