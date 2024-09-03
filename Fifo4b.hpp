#pragma once

#include <atomic>
#include <cassert>
#include <memory>
#include <new>


/**
 * Fifo3a with cached cursors; a threadsafe, efficient circular FIFO
 * with constrained cursors.
 *
 * This Fifo is useful when you need to constrain the cursor ranges. For
 * example say if the sizeof(CursorType) is 8 or 16. The cursors may
 * take on any value up to the fifo's capacity + 1. Furthermore, there
 * are no calculations where an intermediate cursor value is larger then
 * this number. And, finally, the cursors are never negative.
 *
 * The problem that must be resolved is how to distinguish an empty fifo
 * from a full one and still meet the above constraints. First, define
 * an empty fifo as when the pushCursor and popCursor are equal. We
 * cannot define a full fifo as pushCursor == popCursor + capacity as we
 * did with most of the others. Firstly, the intermediate value
 * popCursor + capacity can overflow if a signed cursor is used and if
 * the cursors are constrained to [0..capacity) there is no distiction
 * between the full definition and the empty definition.
 *
 * To resolve this we introduce the idea of a sentinal element by
 * allocating one more element than the capacity of the fifo and define
 * a full fifo as when the cursors are "one apart". That is,
 *
 * @code
 *    | pushCursor < popCursor:  pushCursor == popCursor - 1
 *    | popCursor < pushCursor:  popCursor == pushCursor - capacity
 *    |                   else:  false
 * @endcode
 */
template<typename T, typename Alloc = std::allocator<T>>
class Fifo4b : private Alloc
{
public:
    using value_type = T;
    using allocator_traits = std::allocator_traits<Alloc>;
    using size_type = typename allocator_traits::size_type;

    explicit Fifo4b(size_type capacity, Alloc const& alloc = Alloc{})
        : Alloc{alloc}
        , capacity_{capacity}
        , ring_{allocator_traits::allocate(*this, capacity + 1)}
    {}

    ~Fifo4b() {
        // TODO fix shouldn't matter for benchmark since it waits until
        // the fifo is empty and only need if destructors have side
        // effects.
        // while(not empty()) {
        //     ring_[popCursor_ & mask_].~T();
        //     ++popCursor_;
        // }
        allocator_traits::deallocate(*this, ring_, capacity_ + 1);
    }

    /// Returns the number of elements in the fifo
    auto size() const noexcept {
        auto pushCursor = pushCursor_.load(std::memory_order_relaxed);
        auto popCursor = popCursor_.load(std::memory_order_relaxed);
        if (popCursor <= pushCursor) {
            return pushCursor - popCursor;
        } else {
            return capacity_ - (popCursor - (pushCursor + 1));
        }
    }

    /// Returns whether the container has no elements
    auto empty() const noexcept {
        auto pushCursor = pushCursor_.load(std::memory_order_relaxed);
        auto popCursor = popCursor_.load(std::memory_order_relaxed);
        return empty(pushCursor, popCursor);
    }

    /// Returns whether the container has capacity() elements
    auto full() const noexcept {
        auto pushCursor = pushCursor_.load(std::memory_order_relaxed);
        auto popCursor = popCursor_.load(std::memory_order_relaxed);
        return full(pushCursor, popCursor);
    }

    /// Returns the number of elements that can be held in the fifo
    auto capacity() const noexcept { return capacity_; }


    /// Push one object onto the fifo.
    /// @return `true` if the operation is successful; `false` if fifo is full.
    auto push(T const& value) {
        auto pushCursor = pushCursor_.load(std::memory_order_relaxed);
        if (full(pushCursor, popCursorCached_)) {
            popCursorCached_ = popCursor_.load(std::memory_order_acquire);
            if (full(pushCursor, popCursorCached_)) {
                return false;
            }
        }

        new (&ring_[pushCursor]) T(value);
        if (pushCursor == capacity_) {
            pushCursor_.store(0, std::memory_order_release);
        } else {
            pushCursor_.store(pushCursor + 1, std::memory_order_release);
        }
        return true;
    }

    /// Pop one object from the fifo.
    /// @return `true` if the pop operation is successful; `false` if fifo is empty.
    auto pop(T& value) {
        auto popCursor = popCursor_.load(std::memory_order_relaxed);
        if (empty(pushCursorCached_, popCursor)) {
            pushCursorCached_ = pushCursor_.load(std::memory_order_acquire);
            if (empty(pushCursorCached_, popCursor)) {
                return false;
            }
        }

        value = ring_[popCursor];
        ring_[popCursor].~T();
        if (popCursor == capacity_) {
            popCursor_.store(0, std::memory_order_release);
        } else {
            popCursor_.store(popCursor + 1, std::memory_order_release);
        }
        return true;
    }

private:
    auto full(size_type pushCursor, size_type popCursor) const noexcept {
        if (pushCursor < popCursor) {
            return pushCursor == popCursor - 1;
        } else if (popCursor < pushCursor) {
            return  popCursor == pushCursor - capacity_;
        } else {
            return false;
        }
    }
    static auto empty(size_type pushCursor, size_type popCursor) noexcept {
        return pushCursor == popCursor;
    }

private:
    size_type capacity_;
    T* ring_;

    using CursorType = std::atomic<size_type>;
    static_assert(CursorType::is_always_lock_free);

    // See Fifo3 for reason std::hardware_destructive_interference_size is not used directly
    static constexpr auto hardware_destructive_interference_size = size_type{64};

    /// Loaded and stored by the push thread; loaded by the pop thread
    alignas(hardware_destructive_interference_size) CursorType pushCursor_;

    /// Exclusive to the push thread
    alignas(hardware_destructive_interference_size) size_type popCursorCached_{};

    /// Loaded and stored by the pop thread; loaded by the push thread
    alignas(hardware_destructive_interference_size) CursorType popCursor_;

    /// Exclusive to the pop thread
    alignas(hardware_destructive_interference_size) size_type pushCursorCached_{};

    // Padding to avoid false sharing with adjacent objects
    char padding_[hardware_destructive_interference_size - sizeof(size_type)];
};
