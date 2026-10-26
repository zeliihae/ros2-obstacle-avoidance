// ROS gerektirmeyen birim test:  g++ -std=c++17 -Iinclude test/test_state_machine.cpp -o t && ./t
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "obstacle_avoidance/avoidance_state_machine.hpp"

using namespace obstacle_avoidance;

static int failures = 0;
#define CHECK(cond) \
  do { if (!(cond)) { std::printf("FAIL  %s:%d  %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

// Başlangıçtaki DUR bekleme süresini geçip makineyi SERBEST'e getirir.
static AvoidanceStateMachine make_free_machine(double & t)
{
  AvoidanceStateMachine sm{Config{}};
  t = 0.0;
  sm.update(8.0, t);                  // ilk veri -> DUR (güvenli başlangıç)
  t += 1.1;
  sm.update(8.0, t);                  // bekleme bitti -> YAVASLA
  t += 0.2;
  sm.update(8.0, t);                  // -> SERBEST
  return sm;
}

int main()
{
  const double kInf = std::numeric_limits<double>::infinity();
  double t = 0.0;

  {  // 1) Güvenli başlangıç ve serbest alana geçiş
    AvoidanceStateMachine sm{Config{}};
    CHECK(sm.state() == State::DUR);
    sm.update(8.0, 0.0);
    CHECK(sm.state() == State::DUR);          // hold süresi dolmadı
    sm.update(8.0, 1.1);
    CHECK(sm.state() == State::YAVASLA);
    sm.update(8.0, 1.3);
    CHECK(sm.state() == State::SERBEST);
  }
  {  // 2) Kademeli yaklaşım: SERBEST -> YAVASLA -> KACIN
    auto sm = make_free_machine(t);
    CHECK(sm.state() == State::SERBEST);
    sm.update(1.64, t += 0.17);
    CHECK(sm.state() == State::YAVASLA);      // raporun 1.64 m örneği
    sm.update(0.95, t += 0.17);
    CHECK(sm.state() == State::KACIN);
  }
  {  // 3) Histerezis: sınır değerde titreşim olmamalı
    auto sm = make_free_machine(t);
    sm.update(1.5, t += 0.17);                // YAVASLA
    sm.update(2.05, t += 0.17);               // 2.0 üstü ama histerezis içinde
    CHECK(sm.state() == State::YAVASLA);
    sm.update(2.15, t += 0.17);
    CHECK(sm.state() == State::SERBEST);
    sm.update(0.9, t += 0.17);                // KACIN
    CHECK(sm.state() == State::KACIN);
    sm.update(1.05, t += 0.17);               // 1.0 üstü ama 1.1 altı -> kal
    CHECK(sm.state() == State::KACIN);
    sm.update(1.2, t += 0.17);
    CHECK(sm.state() == State::YAVASLA);
  }
  {  // 4) Ani engel: SERBEST -> DUR, bekleme, sonra KACIN
    auto sm = make_free_machine(t);
    sm.update(0.3, t += 0.17);
    CHECK(sm.state() == State::DUR);
    sm.update(0.3, t += 0.5);
    CHECK(sm.state() == State::DUR);          // 1.0 s dolmadı
    sm.update(0.3, t += 0.6);
    CHECK(sm.state() == State::KACIN);        // yerinde dönüş başlar
    sm.update(0.3, t += 0.17);
    CHECK(sm.state() == State::KACIN);        // DUR'a geri dönmez
  }
  {  // 5) -inf (min menzilden yakın engel) ve NaN => DUR
    auto sm = make_free_machine(t);
    sm.update(-kInf, t += 0.17);
    CHECK(sm.state() == State::DUR);
    auto sm2 = make_free_machine(t);
    sm2.update(std::numeric_limits<double>::quiet_NaN(), t += 0.17);
    CHECK(sm2.state() == State::DUR);
  }
  {  // 6) +inf => engel yok
    auto sm = make_free_machine(t);
    sm.update(kInf, t += 0.17);
    CHECK(sm.state() == State::SERBEST);
  }
  {  // 7) DUR sonrası engel kalktıysa doğrudan YAVASLA
    auto sm = make_free_machine(t);
    sm.update(0.3, t += 0.17);
    sm.update(5.0, t += 1.2);
    CHECK(sm.state() == State::YAVASLA);
  }
  {  // 8) force_stop (sensör kaybı)
    auto sm = make_free_machine(t);
    sm.force_stop(t += 1.0);
    CHECK(sm.state() == State::DUR);
    sm.update(8.0, t += 0.5);
    CHECK(sm.state() == State::DUR);          // hold yeniden başladı
  }
  {  // 9) Config doğrulama + string dönüşümü
    Config bad; bad.stop_dist = 1.5;          // stop > avoid
    CHECK(!bad.valid());
    CHECK(Config{}.valid());
    bool ok = true;
    CHECK(state_from_string("KACIN", &ok) == State::KACIN && ok);
    CHECK(state_from_string("???", &ok) == State::DUR && !ok);
  }

  if (failures == 0) {std::puts("TUM TESTLER GECTI");}
  return failures == 0 ? 0 : 1;
}