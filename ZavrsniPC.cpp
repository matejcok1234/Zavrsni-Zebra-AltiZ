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
#include <boost/asio.hpp>
#include <boost/endian/conversion.hpp>
#include <mil.h>


#pragma comment(lib, "mil.lib")
#pragma comment(lib, "milim.lib")
#pragma comment(lib, "Ws2_32.lib")

const std::string ROBOT_IP_ADRESA = "192.168.40.27";
const int ROBOT_NAREDBENI_PORT = 30002;
const int ROBOT_PODATKOVNI_PORT = 30003;
const size_t VELICINA_ROBOTSKOG_PAKETA = 1116;

const double SKENER_FPS = 48.79;

using boost::asio::ip::tcp;

struct VezaSaRobotom {
    boost::asio::io_context io_context;
    tcp::socket uticnica{ io_context };
    std::vector<char> spremnik_podataka;
};


// Deklaracije funkcija
bool posaljiNaredbu(const std::string& ip, int port, const std::string& poruka);
bool Komunikacija_robot(VezaSaRobotom& veza);
bool TCP_poze(VezaSaRobotom& veza, std::vector<double>& poza);
bool Alociranje(MIL_ID MilSystem, MIL_ID& MilSpremnik);
bool SnimiPodatkeDubine(MIL_ID MilDigitalizator, MIL_ID MilSpremnik, std::vector<double>& izlazniSpremnik, MIL_INT& velicinaX, MIL_INT& velicinaY);
std::vector<double> Dohvati_pozu(const std::string& imeTocke);


int main() {
    std::cout << "--- 3D REKONSTRUKCIJA - COK - ZAVRSNI ---\n";
    auto tockaA_mm = Dohvati_pozu("A (Pocetna tocka)");
    auto tockaB_mm = Dohvati_pozu("B (Zavrsna tocka)");

    double rezolucija = 0.0;
    std::cout << "\nUnesite zeljeni razmak izmedu frameova (u mm): ";
    std::cin >> rezolucija;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (rezolucija <= 0) {
        std::cerr << "Greska: Razmak izmedu okvira mora biti pozitivan broj." << std::endl;
        return EXIT_FAILURE;
    }

    double udaljenost_mm = std::sqrt(
        std::pow(tockaB_mm[0] - tockaA_mm[0], 2) +
        std::pow(tockaB_mm[1] - tockaA_mm[1], 2) +
        std::pow(tockaB_mm[2] - tockaA_mm[2], 2)
    );

    int maksBrojOkvira = static_cast<int>(std::round(udaljenost_mm / rezolucija));

    double brzina_robota_mm_s = rezolucija * SKENER_FPS;
    double brzina_robota_m_s = brzina_robota_mm_s / 1000.0;

    std::cout << "\n--- Automatski izracunati parametri ---" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Ukupna udaljenost skeniranja: " << udaljenost_mm << " mm" << std::endl;
    std::cout << "Ciljana rezolucija: " << rezolucija << " mm/okvir" << std::endl;
    std::cout << "Brzina skenera: " << SKENER_FPS << " FPS" << std::endl;
    std::cout << "Izracunata brzina robota: " << brzina_robota_mm_s << " mm/s (" << std::setprecision(4) << brzina_robota_m_s << " m/s)" << std::endl;
    std::cout << "Izracunati broj okvira za snimanje: " << maksBrojOkvira << std::endl;
    std::cout << "--------------------------------------\n" << std::endl;

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
    if (!posaljiNaredbu(ROBOT_IP_ADRESA, ROBOT_NAREDBENI_PORT, skriptaPomakA.str())) {
        std::cerr << "Greska: Nije uspjelo slanje naredbe za pomak na tocku A. Izlazim." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Naredba za pomak na tocku A poslana. Pricekajte da se robot zaustavi." << std::endl;
    std::cout << "Pritisnite Enter za pocetak skeniranja i pomaka na tocku B..." << std::endl;
    std::cin.get();

    MIL_ID MilApplication = M_NULL, MilSystem = M_NULL;
    MappAlloc(M_NULL, M_DEFAULT, &MilApplication);
    MsysAlloc(M_DEFAULT, M_SYSTEM_DEFAULT, M_DEFAULT, M_DEFAULT, &MilSystem);


    VezaSaRobotom vezaSaRobotom;
    if (!Komunikacija_robot(vezaSaRobotom)) {
        std::cerr << "Nije uspjelo spajanje na robota za podatke. Izlazim.\n";
        MsysFree(MilSystem); MappFree(MilApplication);
        return EXIT_FAILURE;
    }
    std::cout << "Spojen na robota za primanje podataka na " << ROBOT_IP_ADRESA << ":" << ROBOT_PODATKOVNI_PORT << ".\n";

    // Alokacija MIL resursa
    MIL_ID MilSpremnik = M_NULL;
    if (!Alociranje(MilSystem, MilSpremnik)) {
        std::cerr << "Greska pri alociranju kontejnera. Izlazim.\n";
        vezaSaRobotom.uticnica.close(); MsysFree(MilSystem); MappFree(MilApplication);
        return EXIT_FAILURE;
    }
    MIL_ID MilDigitalizator = M_NULL;
    MdigAlloc(MilSystem, M_DEFAULT, MIL_TEXT("M_DEFAULT"), M_DEFAULT, &MilDigitalizator);
    std::cout << "MIL pokrenut.\n";

    // Otvaranje CSV datoteka za spremanje podataka
    std::ofstream datoteka_dubine_izlaz("Kamera.csv");
    std::ofstream datoteka_poze_izlaz("TCP.csv");
    if (!datoteka_dubine_izlaz.is_open() || !datoteka_poze_izlaz.is_open()) {
        std::cerr << "Greska pri otvaranju jedne od CSV datoteka. Izlazim.\n";
        vezaSaRobotom.uticnica.close(); MbufFree(MilSpremnik); MdigFree(MilDigitalizator); MsysFree(MilSystem); MappFree(MilApplication);
        return EXIT_FAILURE;
    }
    datoteka_dubine_izlaz << std::fixed << std::setprecision(6);
    datoteka_poze_izlaz << std::fixed << std::setprecision(8);
    datoteka_poze_izlaz << "frame;x_mm;y_mm;z_mm;rx_rad;ry_rad;rz_rad\n";

    std::cout << "\n2. Pomicanje do tocke B i pocetak skeniranja..." << std::endl;
    if (!posaljiNaredbu(ROBOT_IP_ADRESA, ROBOT_NAREDBENI_PORT, skriptaPomakB.str())) {
        std::cerr << "Greska: Nije uspjelo slanje naredbe za pomak na tocku B. Prekidam.\n";
        datoteka_dubine_izlaz.close(); datoteka_poze_izlaz.close(); vezaSaRobotom.uticnica.close();
        MbufFree(MilSpremnik); MdigFree(MilDigitalizator); MsysFree(MilSystem); MappFree(MilApplication);
        return EXIT_FAILURE;
    }

    std::cout << "Akvizicija pocinje.\n";
    std::cout << "Skeniranje..." << std::flush;

    MIL_INT velicinaX = 0, velicinaY = 0;
    using Clock = std::chrono::high_resolution_clock;
    auto pocetnoVrijeme = Clock::now();

    // Glavna petlja za akviziciju
    for (int okvir = 1; okvir <= maksBrojOkvira; ++okvir) {
        // Snimanje 3D podataka sa skenera
        std::vector<double> spremnik_dubine;
        if (!SnimiPodatkeDubine(MilDigitalizator, MilSpremnik, spremnik_dubine, velicinaX, velicinaY)) {
            break;
        }

        // Dohvaćanje TCP poze robota
        std::vector<double> tcp_poza;
        if (!TCP_poze(vezaSaRobotom, tcp_poza)) {
            // Greška je zanemarena prema zahtjevu
        }

        // Obrada i spremanje podataka o dubini (range data)
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

        // Spremanje podataka o pozi robota
        if (!tcp_poza.empty()) {
            datoteka_poze_izlaz << okvir;
            for (int i = 0; i < 3; ++i) { // Spremi x, y, z u mm
                datoteka_poze_izlaz << ";" << tcp_poza[i] * 1000.0;
            }
            for (int i = 3; i < 6; ++i) { // Spremi rx, ry, rz u radijanima
                datoteka_poze_izlaz << ";" << tcp_poza[i];
            }
            datoteka_poze_izlaz << "\n";
        }
    }
    std::cout << " Zavrseno." << std::endl;

    auto krajnjeVrijeme = Clock::now();
    datoteka_dubine_izlaz.close();
    datoteka_poze_izlaz.close();

    // Ispis statistike
    double protekle_sekunde = std::chrono::duration<double>(krajnjeVrijeme - pocetnoVrijeme).count();
    double stvarni_fps = (maksBrojOkvira > 0 && protekle_sekunde > 0) ? static_cast<double>(maksBrojOkvira) / protekle_sekunde : 0.0;
    std::cout << std::fixed << std::setprecision(2)
        << "\nSnimljeno " << maksBrojOkvira
        << " okvira u " << protekle_sekunde << " sekundi (Prosjek: "
        << stvarni_fps << " FPS)\n";
    std::cout << "Podaci spremljeni u .csv datoteke.\n";

    // Oslobađanje resursa
    vezaSaRobotom.uticnica.close();
    MbufFree(MilSpremnik);
    MdigFree(MilDigitalizator);
    MsysFree(MilSystem);
    MappFree(MilApplication);
    std::cout << "\nSvi resursi oslobodeni. Pritisnite Enter za izlaz.\n";
    std::cin.get();

    return EXIT_SUCCESS;
}

std::vector<double> Dohvati_pozu(const std::string& imeTocke) {
    std::vector<double> poza(6);
    std::cout << "\n--- Unesite koordinate za tocku '" << imeTocke << "' ---" << std::endl;
    std::cout << "Unesite x[mm] y[mm] z[mm] vrijednosti odvojene razmakom:" << std::endl;

    while (true) {
        std::cout << "> ";
        std::string linija;
        std::getline(std::cin, linija);
        std::stringstream ss(linija);
        bool uspjeh = true;

        for (int i = 0; i < 3; ++i) {
            if (!(ss >> poza[i])) {
                uspjeh = false;
                break;
            }
        }

        char visak;
        if (ss >> visak) uspjeh = false;

        if (uspjeh) {
            // Fiksne vrijednosti rotacije
            poza[3] = 2.222;
            poza[4] = 2.220;
            poza[5] = 0.0;
            return poza;
        }

        std::cout << "Neispravan unos. Molimo unesite tocno 3 broja (npr. 100 200 350)." << std::endl;
    }
}

bool TCP_poze(VezaSaRobotom& veza, std::vector<double>& poza) {
    try {
        size_t dostupno_bajtova = veza.uticnica.available();
        if (dostupno_bajtova > 0) {
            std::vector<char> novi_podaci(dostupno_bajtova);
            boost::asio::read(veza.uticnica, boost::asio::buffer(novi_podaci));
            veza.spremnik_podataka.insert(veza.spremnik_podataka.end(), novi_podaci.begin(), novi_podaci.end());
        }

        if (veza.spremnik_podataka.size() < VELICINA_ROBOTSKOG_PAKETA) {
            return false;
        }


        size_t broj_paketa = veza.spremnik_podataka.size() / VELICINA_ROBOTSKOG_PAKETA;
        size_t indeks_pocetka_zadnjeg_paketa = (broj_paketa - 1) * VELICINA_ROBOTSKOG_PAKETA;
        const char* podaci_zadnjeg_paketa = veza.spremnik_podataka.data() + indeks_pocetka_zadnjeg_paketa;
        const size_t pomak_poze = 444; // Offset za 'Actual TCP Pose' u paketu
        poza.resize(6);

        for (int i = 0; i < 6; ++i) {
            uint64_t vrijednost_net;
            std::memcpy(&vrijednost_net, &podaci_zadnjeg_paketa[pomak_poze + i * sizeof(double)], sizeof(double));
            uint64_t vrijednost_host = boost::endian::big_to_native(vrijednost_net); // Konverzija iz big-endian
            std::memcpy(&poza[i], &vrijednost_host, sizeof(double));
        }


        size_t bajtova_za_zadrzati = veza.spremnik_podataka.size() % VELICINA_ROBOTSKOG_PAKETA;
        if (bajtova_za_zadrzati > 0) {
            std::vector<char> preostali_podaci(veza.spremnik_podataka.end() - bajtova_za_zadrzati, veza.spremnik_podataka.end());
            veza.spremnik_podataka = preostali_podaci;
        }
        else {
            veza.spremnik_podataka.clear();
        }
    }
    catch (const boost::system::system_error& e) {
        std::cerr << "\n[GRESKA] Boost.Asio greska pri citanju poze: " << e.code().message() << std::endl;
        return false;
    }
    catch (const std::exception& e) {
        std::cerr << "\n[GRESKA] Opca greska pri citanju poze: " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool posaljiNaredbu(const std::string& ip, int port, const std::string& poruka) {
    try {
        boost::asio::io_context io_context;
        tcp::socket uticnica(io_context);
        tcp::resolver resolver(io_context);
        boost::asio::connect(uticnica, resolver.resolve(ip, std::to_string(port)));
        boost::system::error_code ec;
        boost::asio::write(uticnica, boost::asio::buffer(poruka), ec);
        if (ec) {
            std::cerr << "Greska pri slanju naredbe: " << ec.message() << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Greska pri spajanju ili slanju na port " << port << ": " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool Komunikacija_robot(VezaSaRobotom& veza) {
    try {
        std::cout << "Spajanje na robota za podatke na " << ROBOT_IP_ADRESA << ":" << ROBOT_PODATKOVNI_PORT << "...\n";
        tcp::resolver resolver(veza.io_context);
        boost::asio::connect(veza.uticnica, resolver.resolve(ROBOT_IP_ADRESA, std::to_string(ROBOT_PODATKOVNI_PORT)));
    }
    catch (const std::exception& e) {
        std::cerr << "\nBoost.Asio greska pri spajanju: " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool Alociranje(MIL_ID MilSystem, MIL_ID& MilSpremnik) {
    MilSpremnik = MbufAllocContainer(MilSystem, M_PROC | M_GRAB, M_DEFAULT, M_NULL);
    return (MilSpremnik != M_NULL);
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
    if (!komponentaDubine) return false;

    velicinaX = MbufInquire(komponentaDubine, M_SIZE_X, M_NULL);
    velicinaY = MbufInquire(komponentaDubine, M_SIZE_Y, M_NULL);
    MIL_INT64 ukupno = static_cast<MIL_INT64>(velicinaX) * velicinaY;
    if (velicinaX <= 0 || velicinaY <= 0) return false;

    izlazniSpremnik.resize(static_cast<size_t>(ukupno));

    MIL_INT tip_podataka = MbufInquire(komponentaDubine, M_DATA_TYPE, M_NULL);
    MIL_INT bitova = MbufInquire(komponentaDubine, M_SIZE_BIT, M_NULL);


    if (tip_podataka == M_UNSIGNED && bitova == 16) {
        std::vector<MIL_UINT16> privremeniSpremnik(static_cast<size_t>(ukupno));
        MbufGet(komponentaDubine, privremeniSpremnik.data());
        for (MIL_INT64 i = 0; i < ukupno; ++i) {
            izlazniSpremnik[static_cast<size_t>(i)] = static_cast<double>(privremeniSpremnik[static_cast<size_t>(i)]);
        }
    }
    else {
        return false;
    }
    return true;
}

