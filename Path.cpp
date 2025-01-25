#include "Path.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <map>
#include <algorithm>

Path::Path(TerrainMap& m, std::string name_in, Point start_in, Point finish_in) : map(m), name(name_in), start(start_in), ciel(finish_in) {};

// Bod je legalny ak je na mape a nebol prejdeny
bool Legalne(Point p, const TerrainMap& map, const std::map<Point, bool>& pozrete) {
    return map.legalne_suradnice(p) && pozrete.find(p) == pozrete.end();
}

// BFS pre lietadlo
bool lietadlo::find() {
    std::queue<Point> q;
    std::map<Point, Point> predchadzajuca;
    std::map<Point, bool> pozrete;

    q.push(start);
    pozrete[start] = true;

    std::vector<Point> smery = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

    while (!q.empty()) {
        Point aktualna_poloha = q.front();
        q.pop();

        // Vytvor cestu, ak sa dosiahol ciel
        if (aktualna_poloha == ciel) {
            path.push_back(ciel);
            while (aktualna_poloha != start) {
                aktualna_poloha = predchadzajuca[aktualna_poloha];
                path.push_back(aktualna_poloha);
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        // Pozri vsetky susedne policka
        for (const Point& d : smery) {
            Point susedne_policko = aktualna_poloha + d;
            if (Legalne(susedne_policko, map, pozrete)) {
                q.push(susedne_policko);
                pozrete[susedne_policko] = true;
                predchadzajuca[susedne_policko] = aktualna_poloha;
            }
        }
    }
    return false;
}

// BFS pre lod
bool lod::find() {
    std::queue<Point> q;
    std::map<Point, Point> predchadzajuca;
    std::map<Point, bool> pozrete;

    q.push(start);
    pozrete[start] = true;

    std::vector<Point> smery = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

    while (!q.empty()) {
        Point aktualna_poloha = q.front();
        q.pop();

        // vytvor cestu, ak sa dosiahol ciel
        if (aktualna_poloha == ciel) {
            path.push_back(ciel);
            while (aktualna_poloha != start) {
                aktualna_poloha = predchadzajuca[aktualna_poloha];
                path.push_back(aktualna_poloha);
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        // Pozri vsetky susedne policka
        for (const Point& d : smery) {
            Point susedne_policko = aktualna_poloha + d;
            if (Legalne(susedne_policko, map, pozrete) && (map.alt(susedne_policko) < 0 || susedne_policko == ciel)) {
                q.push(susedne_policko);
                pozrete[susedne_policko] = true;
                predchadzajuca[susedne_policko] = aktualna_poloha;
            }
        }
    }
    return false;
}

// BFSpre auto
bool cesta::find() {
    std::queue<Point> q;
    std::map<Point, Point> predchadzajuca;
    std::map<Point, bool> pozrete;

    q.push(start);
    pozrete[start] = true;

    std::vector<Point> smery = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

    while (!q.empty()) {
        Point aktualna_poloha = q.front();
        q.pop();

        // vytvor cestu, ak sa dosiahol ciel
        if (aktualna_poloha == ciel) {
            path.push_back(ciel);
            while (aktualna_poloha != start) {
                aktualna_poloha = predchadzajuca[aktualna_poloha];
                path.push_back(aktualna_poloha);
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        // pozri vsetky susedne policka
        for (const Point& d : smery) {
            Point susedne_policko = aktualna_poloha + d;
            if (Legalne(susedne_policko, map, pozrete) && (map.alt(susedne_policko) > 0 || susedne_policko == ciel)) {
                double vzdialenost = (susedne_policko - aktualna_poloha).length();
                double sklon = std::abs(map.alt(susedne_policko) - map.alt(aktualna_poloha)) / (vzdialenost * 1000); // Convert vzdialenost to meters
                if (sklon < 0.06) {
                    q.push(susedne_policko);
                    pozrete[susedne_policko] = true;
                    predchadzajuca[susedne_policko] = aktualna_poloha;
                }
            }
        }
    }
    return false;
}

// BFS pre auto a trajekt
bool trajekt::find() {
    std::queue<Point> q;
    std::map<Point, Point> predchadzajuca;
    std::map<Point, bool> pozrete;

    q.push(start);
    pozrete[start] = true;

    std::vector<Point> smery = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

    while (!q.empty()) {
        Point aktualna_poloha = q.front();
        q.pop();

        // vytvor cestu, ak sa dosiahol ciel
        if (aktualna_poloha == ciel) {
            path.push_back(ciel);
            while (aktualna_poloha != start) {
                aktualna_poloha = predchadzajuca[aktualna_poloha];
                path.push_back(aktualna_poloha);
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        // pozri vsetky susedne policka
        for (const Point& d : smery) {
            Point susedne_policko = aktualna_poloha + d;
            if (Legalne(susedne_policko, map, pozrete)) {
                double vzdialenost = (susedne_policko - aktualna_poloha).length();
                double sklon = std::abs(map.alt(susedne_policko) - map.alt(aktualna_poloha)) / (vzdialenost * 1000); // Convert vzdialenost to meters
                if (map.alt(susedne_policko) < 0 || sklon < 0.06) {
                    q.push(susedne_policko);
                    pozrete[susedne_policko] = true;
                    predchadzajuca[susedne_policko] = aktualna_poloha;
                }
            }
        }
    }
    return false;
}

// BFS pre vlak
bool vlak::find() {
    std::queue<Point> q;
    std::map<Point, Point> predchadzajuca;
    std::map<Point, bool> pozrete;
    std::map<Point, double> total_slope;

    q.push(start);
    pozrete[start] = true;
    total_slope[start] = 0.0;

    std::vector<Point> smery = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

    while (!q.empty()) {
        Point aktualna_poloha = q.front();
        q.pop();

        // vytvor cestu, ak sa dosiahol ciel
        if (aktualna_poloha == ciel) {
            path.push_back(ciel);
            while (aktualna_poloha != start) {
                aktualna_poloha = predchadzajuca[aktualna_poloha];
                path.push_back(aktualna_poloha);
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        // pozri vsetky susedne policka
        for (const Point& d : smery) {
            Point susedne_policko = aktualna_poloha + d;
            if (Legalne(susedne_policko, map, pozrete) && map.alt(susedne_policko) > 0) {
                double vzdialenost = (susedne_policko - aktualna_poloha).length();
                double sklon = std::abs(map.alt(susedne_policko) - map.alt(aktualna_poloha)) / (vzdialenost * 1000); // Convert vzdialenost to meters
                double new_total_slope = total_slope[aktualna_poloha] + sklon;

                if (sklon < 0.04 && (!pozrete[susedne_policko] || new_total_slope < total_slope[susedne_policko])) {
                    q.push(susedne_policko);
                    pozrete[susedne_policko] = true;
                    predchadzajuca[susedne_policko] = aktualna_poloha;
                    total_slope[susedne_policko] = new_total_slope;
                }
            }
        }
    }
    return false;
}

//koniec algoritmov

void Path::printStats() const {
    bool land = false;
    bool water = false;
    double length = 0.0;
    double alt = 0.0;

    if (path.size() == 0) {
        std::cout << "Path empty." << std::endl;
        return;
    }

    int max_alt = map.alt(path[0]);

    for (int i=1; i<path.size(); ++i) {
        Point u = path[i];
        Point u_prev = path[i-1];
        if (i < path.size() - 1 && map.alt(u) > 0) land = true;
        if (map.alt(u) < 0) water = true;
        length += (u - u_prev).length();
        alt += std::abs(map.alt(u) - map.alt(u_prev));
        if (map.alt(u) > max_alt) max_alt = map.alt(u);
    }

    std::cout << "Path designated start = [" << start.x << ", " << start.y << "], ciel = [" << ciel.x << ", " << ciel.y << "]" << std::endl;

    if (path[0] != start)
        std::cout << "First point on path [" << path[0].x << ", " << path[0].y << "] does not correspond to the designated starting point [" << start.x << ", " << start.y << "] !" << std::endl;

    if (path[path.size()-1] != ciel)
        std::cout << "Last point on path [" << path[path.size()-1].x << ", " << path[path.size()-1].y << "] does not correspond to the designated ciel point [" << ciel.x << ", " << ciel.y << "] !" << std::endl;

    if (land) std::cout << "Path type: land" << std::endl;
    if (water) std::cout << "Path type: water" << std::endl;
    std::cout << "Path length: " << length << " km" << std::endl;
    std::cout << "Total elevation gain + loss: " << alt << " m" << std::endl;
    std::cout << "Max. elevation: " << max_alt << " m" << std::endl;
}

std::string Path::getName() const {
    return name;
}

void Path::saveToFile() const {
    std::ofstream output(name + ".dat");

    if (!output) throw std::runtime_error("Cannot open file " + name + ".dat");

    for (Point u : path) {
        output << u.x << " " << u.y << std::endl;
    }
}