#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>
#include <array>
#include <cstring>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <cmath>
#include <deque>
#include <mutex>
#include <atomic>

#include <boost/asio.hpp>
#include <boost/endian/conversion.hpp>

#include <mil.h>

#pragma comment(lib, "mil.lib")
#pragma comment(lib, "milim.lib")
#pragma comment(lib, "Ws2_32.lib")

// ----------------------------- Konstante -----------------------------

const std::string ROBOT_IP_ADRESA = "192.168.40.27";
const int ROBOT_NAREDBENI_PORT = 30002;
const int ROBOT_PODATKOVNI_PORT = 30003;
const size_t VELICINA_ROBOTSKOG_PAKETA = 1116;

const double SKENER_FPS = 48.79;
const double REZOLUCIJA_MM = 0.5; // FIKSNO

using boost::asio::ip::tcp;

// ----------------------------- Strukture -----------------------------

struct VezaSaRobotom {
    boost::asio::io_context io_context;
    tcp::socket uticnica{ io_context };
    std::vector<char> spremnik_podataka;
};

struct PoseSample {
    std::array<double, 6> p{}; // x,y,z (m), rx, ry, rz (rad)
    std::chrono::steady_clock::time_point t; // vrijeme prijema paketa
};

// Kruzni spremnik s zaštitom
struct PoseBuffer {
    std::deque<PoseSample> q;
    std::mutex mtx;
    size_t max_keep = 2000; // cca 16 s na 125 Hz

    void push(const PoseSample& s) {
        std::lock_guard<std::mutex> lk(mtx);
        q.push_back(s);
        while (q.size() > max_keep) q.pop_front();
    }

    // vrati pozu najbližu vremenu t; true ako postoji
    bool nearest(const std::chrono::steady_clock::time_point& t, PoseSample& out) {
        std::lock_guard<std::mutex> lk(mtx);
        if (q.empty()) return false;
        size_t lo = 0, hi = q.size();
        while (lo < hi) {
            size_t mid = (lo + hi) / 2;
            if (q[mid].t < t) lo = mid + 1; else hi = mid;
        }
        if (lo == 0) { out = q.front(); return true; }
        if (lo >= q.size()) { out = q.back(); return true; }
        auto& a = q[lo - 1]; auto& b = q[lo];
        out = (t - a.t <= b.t - t) ? a : b;
        return true;
    }
};

// ----------------------------- Deklaracije -----------------------------

bool posaljiNaredbu(const std::string& ip, int port, const std::string& poruka);
bool Komunikacija_robot(VezaSaRobotom& veza);
bool Alociranje(MIL_ID MilSystem, MIL_ID& MilSpremnik);
bool SnimiPodatkeDubine(MIL_ID MilDigitalizator, MIL_ID MilSpremnik, std::vector<double>& izlazniSpremnik, MIL_INT& velicinaX, MIL_INT& velicinaY);
std::vector<double> Dohvati_pozu(const std::string& imeTocke);

static bool ParseLastPacketPose(const std::vector<char>& buf, std::array<double, 6>& poseOut);
void tcp_read_loop(VezaSaRobotom& veza, PoseBuffer& pb, std::atomic<bool>& running,
    std::ofstream* tcp_csv,
    std::chrono::steady_clock::time_point t0);

// ----------------------------- MAIN -----------------------------

int main() {
    std::cout << "--- 3D REKONSTRUKCIJA - COK - ZAVRSNI ---\n";
    std::cout << "FIKSNA rezolucija: " << REZOLUCIJA_MM << " mm/okvir\n";

    auto tockaA_mm = Dohvati_pozu("A (Pocetna tocka)");
    auto tockaB_mm = Dohvati_pozu("B (Zavrsna tocka)");

    double rezolucija = REZOLUCIJA_MM;

    double udaljenost_mm = std::sqrt(
        std::pow(tockaB_mm[0] - tockaA_mm[0], 2) +
        std::pow(tockaB_mm[1] - tockaA_mm[1], 2) +
        std::pow(tockaB_mm[2] - tockaA_mm[2], 2)
    );

    int maksBrojOkvira = static_cast<int>(std::round(udaljenost_mm / rezolucija));

    double brzina_robota_mm_s = rezolucija * SKENER_FPS;
    double brzina_robota_m_s = brzina_robota_mm_s / 1000.0;


    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Ukupna udaljenost skeniranja: " << udaljenost_mm << " mm" << std::endl;
    std::cout << "Rezolucija: " << rezolucija << " mm/okvir" << std::endl;
    std::cout << "Izracunata brzina robota: " << brzina_robota_mm_s << " mm/s (" << std::setprecision(4) << brzina_robota_m_s << " m/s)" << std::endl;
    std::cout << "Izracunati broj okvira za snimanje: " << maksBrojOkvira << std::endl;

    auto uMetre = [](std::vector<double> v) {
        for (int i = 0; i < 3; ++i) v[i] /= 1000.0;
        return v;
        };
    auto tockaA_m = uMetre(tockaA_mm);
    auto tockaB_m = uMetre(tockaB_mm);

    std::ostringstream skriptaPomakA, skriptaPomakB;
    skriptaPomakA << std::fixed << std::setprecision(6)
        << "movel(p[" << tockaA_m[0] << "," << tockaA_m[1] << "," << tockaA_m[2] << ","
        << tockaA_m[3] << "," << tockaA_m[4] << "," << tockaA_m[5] << "], a=1.0, v=0.1)\n";

    skriptaPomakB << std::fixed << std::setprecision(6)
        << "movel(p[" << tockaB_m[0] << "," << tockaB_m[1] << "," << tockaB_m[2] << ","
        << tockaB_m[3] << "," << tockaB_m[4] << "," << tockaB_m[5] << "], a=0.1, v=" << brzina_robota_m_s << ")\n";

    std::cout << "1. Pomicanje na tocku A..." << std::endl;
    posaljiNaredbu(ROBOT_IP_ADRESA, ROBOT_NAREDBENI_PORT, skriptaPomakA.str());
    std::cout << "Naredba za pomak na tocku A poslana. Pricekajte da se robot zaustavi." << std::endl;
    std::cout << "Pritisnite Enter za pocetak skeniranja i pomaka na tocku B..." << std::endl;
    std::cin.get();

    MIL_ID MilApplication = M_NULL, MilSystem = M_NULL;
    MappAlloc(M_NULL, M_DEFAULT, &MilApplication);
    MsysAlloc(M_DEFAULT, M_SYSTEM_DEFAULT, M_DEFAULT, M_DEFAULT, &MilSystem);

    VezaSaRobotom vezaSaRobotom;
    Komunikacija_robot(vezaSaRobotom);

    // Alokacija MIL resursa
    MIL_ID MilSpremnik = M_NULL;
    Alociranje(MilSystem, MilSpremnik);
    MIL_ID MilDigitalizator = M_NULL;
    MdigAlloc(MilSystem, M_DEFAULT, MIL_TEXT("M_DEFAULT"), M_DEFAULT, &MilDigitalizator);
    std::cout << "MIL pokrenut.\n";

    // Otvaranje CSV datoteka
    std::ofstream datoteka_dubine_izlaz("Kamera.csv");
    std::ofstream datoteka_poze_izlaz("TCP.csv");      // SVAKI TCP sample
    std::ofstream datoteka_okvira_izlaz("Frames.csv"); // Uparivanje framea s najbližom TCP pozom

    datoteka_dubine_izlaz << std::fixed << std::setprecision(6);

    datoteka_poze_izlaz << std::fixed << std::setprecision(8);
    datoteka_poze_izlaz << "t_tcp_ns;x_mm;y_mm;z_mm;rx_rad;ry_rad;rz_rad\n";

    datoteka_okvira_izlaz << std::fixed << std::setprecision(8);
    datoteka_okvira_izlaz << "frame;t_frame_ns;x_mm;y_mm;z_mm;rx_rad;ry_rad;rz_rad\n";

    // -------------------------------- Kontinuirani TCP reader --------------------------------
    PoseBuffer pose_buffer;
    std::atomic<bool> running{ true };
    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();

    std::thread tcp_thread(
        tcp_read_loop,
        std::ref(vezaSaRobotom),
        std::ref(pose_buffer),
        std::ref(running),
        &datoteka_poze_izlaz,
        t0
    );

    // -------------------------------- Kretanje robota na B i akvizicija --------------------------------
    std::cout << "\n2. Pomicanje do tocke B i pocetak skeniranja..." << std::endl;
    posaljiNaredbu(ROBOT_IP_ADRESA, ROBOT_NAREDBENI_PORT, skriptaPomakB.str());

    std::cout << "Akvizicija pocinje.\n";
    std::cout << "Skeniranje..." << std::flush;

    MIL_INT velicinaX = 0, velicinaY = 0;
    auto pocetnoVrijeme = Clock::now();

    // Glavna petlja za akviziciju
    for (int okvir = 1; okvir <= maksBrojOkvira; ++okvir) {
        // Snimanje 3D podataka sa skenera
        std::vector<double> spremnik_dubine;
        SnimiPodatkeDubine(MilDigitalizator, MilSpremnik, spremnik_dubine, velicinaX, velicinaY);

        // Timestamp framea odmah nakon hvatanja
        auto t_frame = Clock::now();
        auto t_frame_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t_frame - t0).count();

        // Obrada i spremanje podataka o dubini (range data) u "Kamera.csv"
        std::stringstream linija_stream;
        linija_stream << std::fixed << std::setprecision(6);
        double x_sredina = static_cast<double>(velicinaX - 1) / 2.0;
        for (MIL_INT y = 0; y < velicinaY; ++y) {
            for (MIL_INT x = 0; x < velicinaX; ++x) {
                MIL_INT64 idx = static_cast<MIL_INT64>(y) * velicinaX + x;
                double centrirani_x = (static_cast<double>(x) - x_sredina) * 0.157468;
                linija_stream << centrirani_x << "," << 0 << "," << (spremnik_dubine[idx] * 0.00581 + 160) << ",1";
                if (y < velicinaY - 1 || x < velicinaX - 1) {
                    linija_stream << ";";
                }
            }
        }
        datoteka_dubine_izlaz << linija_stream.str() << "\n";

        // Uparivanje s najbližom TCP pozom iz buffer-a i zapis u "Frames.csv"
        PoseSample najbliza{};
        bool ima_pozu = pose_buffer.nearest(t_frame, najbliza);

        datoteka_okvira_izlaz << okvir << ";" << t_frame_ns;
        if (ima_pozu) {
            for (int i = 0; i < 3; ++i) datoteka_okvira_izlaz << ";" << (najbliza.p[i] * 1000.0); // mm
            for (int i = 3; i < 6; ++i) datoteka_okvira_izlaz << ";" << najbliza.p[i];            // rad
        }
        else {
            datoteka_okvira_izlaz << ";" << ";" << ";" << ";" << ";" << ";";
        }
        datoteka_okvira_izlaz << "\n";
    }
    std::cout << " Zavrseno." << std::endl;

    auto krajnjeVrijeme = std::chrono::steady_clock::now();

    // Zaustavi TCP thread i zatvori socket
    running.store(false, std::memory_order_relaxed);
    vezaSaRobotom.uticnica.close();
    if (tcp_thread.joinable()) tcp_thread.join();

    // Zatvori CSV-ove
    datoteka_dubine_izlaz.close();
    datoteka_poze_izlaz.close();
    datoteka_okvira_izlaz.close();

    std::cout << "Podaci spremljeni\n";

    // Oslobađanje resursa
    MbufFree(MilSpremnik);
    MdigFree(MilDigitalizator);
    MsysFree(MilSystem);
    MappFree(MilApplication);
    std::cout << "\nSvi resursi oslobodeni. Pritisnite Enter za izlaz.\n";
    std::cin.get();

    return EXIT_SUCCESS;
}

// ----------------------------- Implementacije pomoćnih funkcija -----------------------------

std::vector<double> Dohvati_pozu(const std::string& imeTocke) {
    std::vector<double> poza(6);
    std::cout << "\n--- Unesite koordinate za tocku '" << imeTocke << "' ---" << std::endl;
    std::cout << "Unesite x[mm] y[mm] z[mm] vrijednosti odvojene razmakom:" << std::endl;

    std::cout << "> ";
    std::string linija;
    std::getline(std::cin, linija);
    std::stringstream ss(linija);

    for (int i = 0; i < 3; ++i) {
        ss >> poza[i];
    }
    // Fiksne vrijednosti rotacije (rad)
    poza[3] = 2.222;
    poza[4] = 2.220;
    poza[5] = 0.0;
    return poza;
}

bool posaljiNaredbu(const std::string& ip, int port, const std::string& poruka) {
    boost::asio::io_context io_context;
    tcp::socket uticnica(io_context);
    tcp::resolver resolver(io_context);
    boost::asio::connect(uticnica, resolver.resolve(ip, std::to_string(port)));
    boost::asio::write(uticnica, boost::asio::buffer(poruka));
    return true;
}

bool Komunikacija_robot(VezaSaRobotom& veza) {
    tcp::resolver resolver(veza.io_context);
    boost::asio::connect(veza.uticnica, resolver.resolve(ROBOT_IP_ADRESA, std::to_string(ROBOT_PODATKOVNI_PORT)));
    return true;
}

bool Alociranje(MIL_ID MilSystem, MIL_ID& MilSpremnik) {
    MilSpremnik = MbufAllocContainer(MilSystem, M_PROC | M_GRAB, M_DEFAULT, M_NULL);
    return true;
}

bool SnimiPodatkeDubine(MIL_ID MilDigitalizator, MIL_ID MilSpremnik, std::vector<double>& izlazniSpremnik, MIL_INT& velicinaX, MIL_INT& velicinaY) {
    MdigGrab(MilDigitalizator, MilSpremnik);

    std::vector<MIL_ID> komponente;
    MbufInquireContainer(MilSpremnik, M_CONTAINER, M_COMPONENT_LIST, komponente);
    MIL_ID komponentaDubine = M_NULL;
    for (auto c : komponente) {
        if (MbufInquire(c, M_COMPONENT_TYPE, M_NULL) == M_COMPONENT_RANGE) {
            komponentaDubine = c;
            break;
        }
    }

    velicinaX = MbufInquire(komponentaDubine, M_SIZE_X, M_NULL);
    velicinaY = MbufInquire(komponentaDubine, M_SIZE_Y, M_NULL);
    MIL_INT64 ukupno = static_cast<MIL_INT64>(velicinaX) * velicinaY;

    izlazniSpremnik.resize(static_cast<size_t>(ukupno));

    std::vector<MIL_UINT16> privremeniSpremnik(static_cast<size_t>(ukupno));
    MbufGet(komponentaDubine, privremeniSpremnik.data());
    for (MIL_INT64 i = 0; i < ukupno; ++i) {
        izlazniSpremnik[static_cast<size_t>(i)] = static_cast<double>(privremeniSpremnik[static_cast<size_t>(i)]);
    }
    return true;
}

// ----------------------------- TCP reader & parser -----------------------------

static bool ParseLastPacketPose(const std::vector<char>& buf, std::array<double, 6>& poseOut) {
    size_t nPackets = buf.size() / VELICINA_ROBOTSKOG_PAKETA;
    size_t start = (nPackets - 1) * VELICINA_ROBOTSKOG_PAKETA;
    const char* p = buf.data() + start;
    const size_t POSE_OFFSET = 444; // Offset za "Actual TCP Pose" u paketu
    for (int i = 0; i < 6; ++i) {
        uint64_t net_u64;
        std::memcpy(&net_u64, p + POSE_OFFSET + i * sizeof(double), sizeof(double));
        uint64_t host_u64 = boost::endian::big_to_native(net_u64);
        double val;
        std::memcpy(&val, &host_u64, sizeof(double));
        poseOut[i] = val;
    }
    return true;
}

void tcp_read_loop(VezaSaRobotom& veza, PoseBuffer& pb, std::atomic<bool>& running,
    std::ofstream* tcp_csv,
    std::chrono::steady_clock::time_point t0)
{
    std::vector<char> acc;
    acc.reserve(VELICINA_ROBOTSKOG_PAKETA * 8);

    boost::system::error_code ec;
    std::array<char, 4096> tmp;

    while (running.load(std::memory_order_relaxed)) {
        size_t n = veza.uticnica.read_some(boost::asio::buffer(tmp), ec);
        auto t_recv = std::chrono::steady_clock::now(); // timestamp za ovaj chunk

        acc.insert(acc.end(), tmp.data(), tmp.data() + n);

        if (acc.size() >= VELICINA_ROBOTSKOG_PAKETA) {
            std::array<double, 6> pose;
            if (ParseLastPacketPose(acc, pose)) {
                PoseSample s;
                s.p = pose;
                s.t = t_recv;
                pb.push(s);

                if (tcp_csv && tcp_csv->is_open()) {
                    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(s.t - t0).count();
                    (*tcp_csv) << ns;
                    for (int i = 0; i < 3; ++i) (*tcp_csv) << ";" << (s.p[i] * 1000.0); // mm
                    for (int i = 3; i < 6; ++i) (*tcp_csv) << ";" << s.p[i];            // rad
                    (*tcp_csv) << "\n";
                }
            }
            // Zadrži samo eventualni nepotpuni rep
            size_t rem = acc.size() % VELICINA_ROBOTSKOG_PAKETA;
            if (rem) {
                std::vector<char> tail(acc.end() - rem, acc.end());
                acc.swap(tail);
            }
            else {
                acc.clear();
            }
        }
    }
}
