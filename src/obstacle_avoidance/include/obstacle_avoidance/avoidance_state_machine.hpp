// avoidance_state_machine.hpp
// Karar mantığı: ROS'a bağımlı OLMAYAN saf C++ state machine.
// Bu sayede g++ ile tek başına test edilebilir (test/test_state_machine.cpp).
#pragma once

#include <cmath>
#include <string>

namespace obstacle_avoidance
{

enum class State { SERBEST, YAVASLA, DUR, KACIN };

inline const char * to_string(State s)
{
  switch (s) {
    case State::SERBEST: return "SERBEST";
    case State::YAVASLA: return "YAVASLA";
    case State::DUR:     return "DUR";
    case State::KACIN:   return "KACIN";
  }
  return "DUR";  // güvenli varsayılan
}

// Bilinmeyen metin gelirse güvenli taraf olan DUR döner.
inline State state_from_string(const std::string & text, bool * ok = nullptr)
{
  State result = State::DUR;
  bool found = true;
  if (text == "SERBEST") {
    result = State::SERBEST;
  } else if (text == "YAVASLA") {
    result = State::YAVASLA;
  } else if (text == "DUR") {
    result = State::DUR;
  } else if (text == "KACIN") {
    result = State::KACIN;
  } else {
    found = false;
  }
  if (ok) {*ok = found;}
  return result;
}

struct Config
{
  double slow_dist = 2.0;       // [m] bu mesafenin altında YAVASLA
  double avoid_dist = 1.0;      // [m] bu mesafenin altında KACIN
  double stop_dist = 0.5;       // [m] bu mesafenin altında DUR (acil fren)
  double hysteresis = 0.1;      // [m] durumdan ÇIKARKEN eklenen pay (titreşimi önler)
  double stop_hold_time = 1.0;  // [s] DUR durumunda minimum bekleme
  double side_margin = 0.1;     // [m] sol/sag farki bundan kucukse varsayilan yon
  int default_turn = 1;         // varsayilan donus yonu: +1 sol, -1 sag

  bool valid() const
  {
    const double margin = std::fmin(avoid_dist - stop_dist, slow_dist - avoid_dist);
    return stop_dist > 0.0 && stop_dist < avoid_dist && avoid_dist < slow_dist &&
           hysteresis >= 0.0 && hysteresis < margin && stop_hold_time >= 0.0 &&
           side_margin >= 0.0 && (default_turn == 1 || default_turn == -1);
  }
};

class AvoidanceStateMachine
{
public:
  AvoidanceStateMachine() = default;
  explicit AvoidanceStateMachine(const Config & cfg)
  : cfg_(cfg), turn_dir_(cfg.default_turn) {}

  State state() const {return state_;}

  // KACIN'a girerken secilen ve KACIN boyunca sabit tutulan donus yonu: +1 sol, -1 sag.
  int turn_direction() const {return turn_dir_;}

  // Sol/sag sektordeki en yakin engel mesafeleri (perception_node'dan).
  void set_side_distances(double left, double right)
  {
    left_ = sanitize_side(left);
    right_ = sanitize_side(right);
    have_sides_ = true;
  }

  // Yan sensor verisi eskidiyse/yoksa cagrilir; yon varsayilana duser.
  void clear_side_distances() {have_sides_ = false;}

  // Yeni mesafe ölçümü ile durumu günceller. now: saniye cinsinden zaman.
  State update(double distance, double now)
  {
    if (!started_) {
      entered_at_ = now;
      started_ = true;
    }
    const double d = sanitize(distance);

    switch (state_) {
      case State::SERBEST:
        if (d <= cfg_.stop_dist) {
          enter(State::DUR, now);
        } else if (d <= cfg_.avoid_dist) {
          enter(State::KACIN, now);
        } else if (d <= cfg_.slow_dist) {
          enter(State::YAVASLA, now);
        }
        break;

      case State::YAVASLA:
        if (d <= cfg_.stop_dist) {
          enter(State::DUR, now);
        } else if (d <= cfg_.avoid_dist) {
          enter(State::KACIN, now);
        } else if (d > cfg_.slow_dist + cfg_.hysteresis) {
          enter(State::SERBEST, now);
        }
        break;

      case State::KACIN:
        // KACIN = yerinde dönüş (lineer hız 0), bu yüzden burada DUR'a
        // geri dönülmez; aksi halde DUR <-> KACIN salınımı oluşurdu.
        if (d > cfg_.avoid_dist + cfg_.hysteresis) {
          enter(State::YAVASLA, now);
        }
        break;

      case State::DUR:
        if (now - entered_at_ >= cfg_.stop_hold_time) {
          enter(d > cfg_.avoid_dist + cfg_.hysteresis ? State::YAVASLA : State::KACIN, now);
        }
        break;
    }
    return state_;
  }

  // Sensör kaybı gibi durumlarda zorla DUR. Her çağrıda bekleme süresi
  // yeniden başlar; yani sensör geri gelince DUR en az stop_hold_time sürer.
  void force_stop(double now)
  {
    started_ = true;
    enter(State::DUR, now);
  }

  double time_in_state(double now) const {return now - entered_at_;}

private:
  // -inf, NaN ve negatif değerler "çok yakın engel" (0 m) sayılır.
  // +inf ise engel yok anlamına gelir ve olduğu gibi bırakılır.
  static double sanitize(double d)
  {
    if (std::isnan(d) || d < 0.0) {return 0.0;}
    return d;
  }

  // Yan mesafe 0 m (kapali/gecersiz) sayilir; +inf ise buyuk bir sonlu deger olur.
  static double sanitize_side(double d)
  {
    if (std::isnan(d) || d < 0.0) {return 0.0;}
    return std::fmin(d, 100.0);
  }

  // Daha bos tarafi sec. Fark kucukse ya da veri yoksa varsayilan yon.
  int choose_turn() const
  {
    if (!have_sides_ || std::fabs(left_ - right_) < cfg_.side_margin) {
      return cfg_.default_turn;
    }
    return left_ > right_ ? 1 : -1;
  }

  void enter(State next, double now)
  {
    if (next == State::KACIN && state_ != State::KACIN) {
      turn_dir_ = choose_turn();  // yon yalnizca KACIN'a girerken belirlenir
    }
    state_ = next;
    entered_at_ = now;
  }

  Config cfg_{};
  int turn_dir_ = 1;
  double left_ = 0.0;
  double right_ = 0.0;
  bool have_sides_ = false;
  State state_ = State::DUR;  // güvenli başlangıç: ilk veri gelene kadar dur
  double entered_at_ = 0.0;
  bool started_ = false;
};

}  // namespace obstacle_avoidance