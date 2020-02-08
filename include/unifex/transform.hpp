/*
 * Copyright 2019-present Facebook, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <unifex/config.hpp>
#include <unifex/receiver_concepts.hpp>
#include <unifex/sender_concepts.hpp>
#include <unifex/type_traits.hpp>
#include <unifex/blocking.hpp>
#include <unifex/get_stop_token.hpp>
#include <unifex/async_trace.hpp>
#include <unifex/type_list.hpp>

#include <functional>
#include <type_traits>

namespace unifex {

namespace detail {

UNIFEX_HIDDEN_FRIEND_NAMESPACE_FOR_BEGIN(transform_operation)

template<typename Source, typename Func, typename Receiver>
class transform_operation;

UNIFEX_HIDDEN_FRIEND_NAMESPACE_FOR_END(transform_operation)

UNIFEX_HIDDEN_FRIEND_NAMESPACE_FOR_BEGIN(transform_receiver)

template <typename Source, typename Func, typename Receiver>
class transform_receiver {
    using operation = transform_operation<Source, Func, Receiver>;

public:
    explicit transform_receiver(operation* op) noexcept
        : op_(op)
    {}

    transform_receiver(transform_receiver&& other) noexcept
        : op_(std::exchange(other.op_, nullptr))
    {}

    template <
        typename... Values,
        typename Result = std::invoke_result_t<Func, Values...>,
        std::enable_if_t<
            std::is_void_v<Result> &&
            std::is_invocable_v<decltype(unifex::set_value), Receiver>, int> = 0>
    void set_value(Values&&... values) && noexcept(
            std::is_nothrow_invocable_v<Func, Values...> &&
            std::is_nothrow_invocable_v<decltype(unifex::set_value), Receiver>) {
        std::invoke((Func&&)op_->func_, (Values&&)values...);
        unifex::set_value((Receiver&&)op_->receiver_);
    }

    template <
        typename... Values,
        typename Result = std::invoke_result_t<Func, Values...>,
        std::enable_if_t<
            !std::is_void_v<Result> &&
            std::is_invocable_v<decltype(unifex::set_value), Receiver, Result>,
            int> = 0>
    void set_value(Values&&... values) && noexcept(
            std::is_nothrow_invocable_v<Func, Values...> &&
            std::is_nothrow_invocable_v<decltype(unifex::set_value), Receiver, Result>) {
        unifex::set_value(
            (Receiver&&)op_->receiver_,
            std::invoke((Func&&)op_->func_, (Values&&)values...));
    }

    template <
        typename Error,
        std::enable_if_t<
            std::is_invocable_v<decltype(unifex::set_error), Receiver, Error>,
            int> = 0>
    void set_error(Error&& error) && noexcept {
        unifex::set_error((Receiver&&)op_->receiver_, (Error&&)error);
    }

    void set_done() && noexcept {
        unifex::set_done((Receiver&&)op_->receiver_);
    }

    template <
        typename CPO,
        std::enable_if_t<!is_receiver_cpo_v<CPO>, int> = 0,
        std::enable_if_t<std::is_invocable_v<CPO, const Receiver&>, int> = 0>
     friend auto tag_invoke(CPO cpo, const transform_receiver& r)
        noexcept(std::is_nothrow_invocable_v<CPO, const Receiver&>)
        -> std::invoke_result_t<CPO, const Receiver&> {
        return std::move(cpo)(r.get_receiver());
    }

    template <typename Visit>
    friend void tag_invoke(
        tag_t<visit_continuations>,
        const transform_receiver& r,
        Visit&& visit) {
        std::invoke(visit, r.get_receiver());
    }

private:
    const Receiver& get_receiver() const noexcept {
        return op_->receiver_;
    }

    operation* op_;
};

UNIFEX_HIDDEN_FRIEND_NAMESPACE_FOR_END(transform_receiver)

UNIFEX_HIDDEN_FRIEND_NAMESPACE_FOR_BEGIN(transform_operation)

template<typename Source, typename Func, typename Receiver>
class transform_operation {
    UNIFEX_NO_UNIQUE_ADDRESS Func func_;
    UNIFEX_NO_UNIQUE_ADDRESS Receiver receiver_;
    UNIFEX_NO_UNIQUE_ADDRESS operation_t<Source, transform_receiver<Source, Func, Receiver>> sourceOp_;

    friend class transform_receiver<Source, Func, Receiver>;

public:
    template<typename Func2, typename Receiver2>
    explicit transform_operation(Source&& source, Func2&& func, Receiver2&& receiver)
        noexcept(std::is_nothrow_constructible_v<Func, Func2> &&
                 std::is_nothrow_constructible_v<Receiver, Receiver2> &&
                 is_nothrow_connectable_v<Source, transform_receiver<Source, Func, Receiver>>)
        : func_((Func2&&)func)
        , receiver_((Receiver2&&)receiver)
        , sourceOp_(unifex::connect((Source&&)source, transform_receiver<Source, Func, Receiver>{ this }))
    {}

    void start() noexcept {
        unifex::start(sourceOp_);
    }
};

UNIFEX_HIDDEN_FRIEND_NAMESPACE_FOR_END(transform_operation)

UNIFEX_HIDDEN_FRIEND_NAMESPACE_FOR_BEGIN(transform_sender)

template <typename Predecessor, typename Func>
struct transform_sender {
  UNIFEX_NO_UNIQUE_ADDRESS Predecessor pred_;
  UNIFEX_NO_UNIQUE_ADDRESS Func func_;

  template <template <typename...> class Tuple>
  struct transform_result {
   private:
    template <typename Result, typename = void>
    struct impl {
      using type = Tuple<Result>;
    };
    template <typename Result>
    struct impl<Result, std::enable_if_t<std::is_void_v<Result>>> {
      using type = Tuple<>;
    };

   public:
    template <typename... Args>
    using apply = typename impl<std::invoke_result_t<Func, Args...>>::type;
  };

  template <
      template <typename...> class Variant,
      template <typename...> class Tuple>
  using value_types = deduplicate_t<typename Predecessor::template value_types<
      Variant,
      transform_result<Tuple>::template apply>>;

  template <template <typename...> class Variant>
  using error_types = typename Predecessor::template error_types<Variant>;

  friend constexpr auto tag_invoke(
      tag_t<blocking>,
      const transform_sender& sender) {
    return blocking(sender.pred_);
  }

  template <
      typename Receiver,
      std::enable_if_t<
        is_connectable_v<
            Predecessor,
            transform_receiver<Predecessor, Func, std::remove_cvref_t<Receiver>>>, int> = 0>
  auto connect(Receiver && receiver) &&
      noexcept(is_nothrow_constructible_v<Op, Predecessor, Func, Receiver>)
      -> transform_operation<Predecessor, Func, std::remove_cvref_t<Receiver>> {
      return transform_operation<Predecessor, Func, std::remove_cvref_t<Receiver>>{
          (Predecessor&&)pred_,
          (Func&&)func_,
          (Receiver&&)receiver};
  }

  template <
      typename Receiver,
      std::enable_if_t<
        std::is_copy_constructible_v<Func> &&
        is_connectable_v<
            const Predecessor&,
            transform_receiver<const Predecessor&, Func, std::remove_cvref_t<Receiver>>>, int> = 0>
  auto connect(Receiver && receiver) const &
      noexcept(is_nothrow_constructible_v<Op, const Predecessor&, const Func&, Receiver>)
      -> transform_operation<const Predecessor&, Func, std::remove_cvref_t<Receiver>> {
      return transform_operation<const Predecessor&, Func, std::remove_cvref_t<Receiver>>{
          pred_,
          func_,
          (Receiver&&)receiver};
  }

  template <
      typename Receiver,
      std::enable_if_t<
        std::is_constructible_v<Func, Func&> &&
        is_connectable_v<
            Predecessor&,
            transform_receiver<Predecessor&, Func, std::remove_cvref_t<Receiver>>>, int> = 0>
  auto connect(Receiver && receiver) &
      noexcept(is_nothrow_constructible_v<Op, Predecessor&, Func&, Receiver>)
      -> transform_operation<Predecessor&, Func, std::remove_cvref_t<Receiver>> {
      return transform_operation<Predecessor, Func, std::remove_cvref_t<Receiver>>{
           pred_,
           func_,
           (Receiver&&)receiver};
  }
};

UNIFEX_HIDDEN_FRIEND_NAMESPACE_FOR_END(transform_sender)

} // namespace detail

template <typename Sender, typename Func>
auto transform(Sender&& predecessor, Func&& func) {
  return detail::transform_sender<std::remove_cvref_t<Sender>, std::decay_t<Func>>{
      (Sender &&) predecessor, (Func &&) func};
}

} // namespace unifex
