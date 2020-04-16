/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "improved_viewer.h"

#include <easy3d/core/point_cloud.h>
#include <easy3d/viewer/camera.h>
#include <easy3d/viewer/drawable_points.h>
#include <easy3d/algo/point_cloud_normals.h>
#include <3rd_party/glfw/include/GLFW/glfw3.h>	// for the KEYs
#include <fstream>

using namespace easy3d;

ImprovedViewer::ImprovedViewer(const std::string& title) : Viewer(title) {
    //Postavke kamere
    //Izvršite izmjenu stavki da vidite kako se kamera ponasa
    camera()->setType(Camera::ORTHOGRAPHIC);
    camera()->setUpVector(vec3(0, -1, 0));
    camera()->setViewDirection(vec3(0,0,1));
    camera()->setSceneRadius(camera()->screenHeight());
    camera()->setPosition(vec3(0, 0, 0));
}


void ImprovedViewer::ucitajTackeIzDatoteke(const std::string &ime)
{
    auto tacke = new PointsDrawable("tacke");
    std::ifstream f(ime);
    float x{}, y{};
    char zarez{};
    std::vector<vec3> points;
    while(f)
    {
        f >> x >> zarez >>  y;
        points.emplace_back(x, y,0);
    }

    tacke->update_vertex_buffer(points);
    this->add_drawable(tacke);
    f.close();
}

void ImprovedViewer::ucitajLinijeIzDatoteke(const std::string &ime, const std::string &imeIndices)
{
    std::ifstream f(ime);
        LinesDrawable * linije = new LinesDrawable("linije");
        float x{}, y{};
        char zarez{};
        int index{};
        std::vector<vec3> points;
        std::vector<uint32_t> indices;
        while(f)
        {
            f >> x >>  zarez >> y;
            points.emplace_back(x, y,0);
        }
        f.close();
        f.open(imeIndices);
        while(f)
        {
            f >> index >> zarez;
            indices.emplace_back(index);
        }
        linije->update_vertex_buffer(points);
        linije->update_index_buffer(indices);
        this->add_drawable(linije);
        f.close();
}

std::string ImprovedViewer::usage() const {
    return ("----------- ImprovedViewer usage------------ \n"
            "Right click generates vertex\n"
            "Key press of n enables input\n"
            "Key press of f enables read\n"
            "------------------------------------------------ \n");
}


bool ImprovedViewer::mouse_press_event(int x, int y, int button, int modifiers)  {
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        //Koja koordinata je fiksirana? z
        //Koja pomocna metoda treba da se tacka vidi?
        float x = 3, y = 3;
        std::cout << "Unesi x i y:\n";
        std::cin >> x >> y;
        this->pts.emplace_back(x,y,0);
        auto nova = new PointsDrawable("tacka");
        nova->update_vertex_buffer({{x,y,0}});
        nova->set_point_size(5);
        this->add_drawable(nova);
        update_rendering();
        camera()->showEntireScene();
        update();
        update_rendering();
        }
   else
        return Viewer::mouse_press_event(x,y,button, modifiers);
}

void ImprovedViewer::iscrtajPoli()
{
    auto P = this->pts;
    int b = 0;
    for(int i = 0; i < P.size(); ++i)
        if((P[i].x < P[b].x) || (P[i].x == P[b].x && P[i].y < P[b].y))
            b = i;

    auto Q = P[b];
    auto poredi = [Q](vec3 R, vec3 S)
    {
        double a = R.x - Q.x;
        double b = R.y - Q.y;
        double c = S.x - Q.x;
        double d = S.y - Q.y;
        double e = a * d - b * c;
        return (e != 0) ?  e > 0 : a*a + b*b < c*c + d*d;
    };
    std::sort(P.begin(), P.end(), poredi);
    pts = P;
}

void ImprovedViewer::ucitajPoliIzDatoteke(const std::string &ime) {
    std::ifstream f(ime);
    pts.resize(0);
    int n;
    f >> n;
    for(int i = 0; i < n; ++i)
    {
        double x;
        double y;
        f >> x >> y;
        pts.push_back({(float)x, (float)y,0});
    }
    f.close();
    iscrtajPoli();
}

double ataux(vec3 P1, vec3 P2, vec3 P3)
{
    return (P2.x - P1.x) * (P3.y - P1.y) - (P3.x - P1.x) * (P2.y - P1.y);
}

bool ImprovedViewer::key_press_event(int key, int modifiers) {
    if (key == GLFW_KEY_N) {
        std::cout<< "Za tacku unesi T, a za liniju nesto sto nije T:";
        char znak = ' ';
        std::cin >> znak;
        if(znak == 'T')
        {
            float x=3, y=3;
            std::cout << "Unesi x i y:\n";
            std::cin >> x >> y;
            auto nova = new PointsDrawable("tacka");
            nova->update_vertex_buffer({{x,y,0}});
            nova->set_point_size(5);
            this->add_drawable(nova);
        }
        else{
            float x1 =3, y1 = 3, x2 = 3, y2 = 3;
            std::cout<<"Unesi x1, y1, x2, y2: \n";
            std::cin >> x1 >> y1 >> x2 >> y2;
            auto nova = new LinesDrawable("linije");
            nova->update_vertex_buffer({{x1,y1,0}, {x2,y2,0}});
            nova->update_index_buffer({1,2});
            this->add_drawable(nova);
        }
        update_rendering();
        camera()->showEntireScene();
        update();
        update_rendering();
    }
    else if (key == GLFW_KEY_K) {
        std::cout << "Unesi T za ucitavanje tacaka, a nesto sto nije T za ucitavanje linija:\n";
        char znak = ' ';
        std::cin>>znak;
        if(znak == 'T')
        {
            std::cout<<"unesi ime datoteke sa tackama: \n";
            std::string dat;
            std::cin>>dat;
            ucitajTackeIzDatoteke(dat);
        }
        else{
            std::cout<<"unesi ime datoteke sa linijama: \n";
            std::string dat1, dat2;
            std::cin>>dat1;
            std::cout<<"unesi ime datoteke sa indices:\n";
            std::cin>>dat2;
            ucitajLinijeIzDatoteke(dat1,dat2);
        }
        update_rendering();
        camera()->showEntireScene();
        update();
        update_rendering();
    }
    else if(key == GLFW_KEY_P)
    {
        iscrtajPoli();
        LinesDrawable *l = new LinesDrawable("poligon");

        std::vector<unsigned int> indi;
        for(unsigned long i = 0; i < pts.size()-1;++i) { // i = 0, i = 1, i = 2
            indi.push_back(i);
            indi.push_back(i+1);
        }

        indi.push_back(pts.size()-1); // 3
        indi.push_back(0);

        for(auto x:indi)std::cout<<x<<std::endl;

        l->update_vertex_buffer(pts);
        l->update_index_buffer(indi);
        this->add_drawable(l);
        update_rendering();
    }
    else if(key == GLFW_KEY_I)
    {
        std::cout<<"Unesi ime datoteke:";
        std::string s;
        std::cin>>s;
        ucitajPoliIzDatoteke(s);
    }
    else if(key == GLFW_KEY_C)
    {
        std::cout<<"Unesi x i y: ";
        float x,y ;
        std::cin>>x>>y;
        vec3 t(x,y,0);
        bool f = false;
        std::vector<vec3> poligon;
        for(int i = 0; i < pts.size(); ++i)
        {
            poligon.push_back(pts[i]);
            i == pts.size() - 1 ? poligon.push_back(pts[0]) : poligon.push_back(pts[i+1]);
        }
        for(int i = 0; i < poligon.size(); ++i)
        {
            int j = i % poligon.size();
            if((poligon[j].y <= t.y && t.y < poligon[i].y && ataux(poligon[j], poligon[i], t) > 0)
            || (poligon[i].y <= t.y && t.y < poligon[j].y && ataux(poligon[i], poligon[j], t)> 0))
                f = !f;
        }
        if(f) std::cout<<"Tacka je unutar poligona\n";
        else std::cout<<"Tacka je van poligona\n";

    }
    else
        return Viewer::key_press_event(key, modifiers);
}

void ImprovedViewer::update_rendering() {
    PointCloud* cloud = dynamic_cast<PointCloud*>(current_model());
    if (cloud == nullptr)
        return;

    // The "normal" property
    auto normals = cloud->get_vertex_property<vec3>("v:normal");
    if (normals) {
        auto drawable = cloud->points_drawable("vertices");
        // Upload the vertex normals to the GPU.
        drawable->update_normal_buffer(normals.vector());
        update();
    }
}


