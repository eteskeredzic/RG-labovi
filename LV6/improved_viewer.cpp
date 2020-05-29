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

std::vector<vec3> ImprovedViewer::convHull()
{
    auto P = pts;
    int b = 0, n = P.size();

    for(int i = 0; i < n; ++i)
    {
        if((P[i].x < P[b].x) || (P[i].x == P[b].x && P[i].y < P[b].y)) b = i;
    }
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
    int k = -1;
    for(int i = 0; i < n; ++i)
    {
        while(k > 0 && ataux(P[k-1], P[k], P[i]) < 0)
            k--;
        k++;
        P[k] = P[i];
    }
    P.resize(k+1);
    return P;
}

std::vector<vec3> ImprovedViewer::areaSearchNaive(double x1, double x2, double y1, double y2)
{
    std::vector<vec3> Q;
    for(int i = 0; i < pts.size(); ++i)
    {
        if(pts[i].x >= x1 && pts[i].x <= x2 && pts[i].y >= y1 && pts[i].y <= y2) Q.emplace_back(pts[i].x,pts[i].y,0);
    }
    return Q;
}

inline int SlucajniBroj (int mini, int maxi){ // funkcija za nasumicni broj izmedju 2 broja koja NIJE 'biased'
    int n = maxi - mini + 1;
    int ost = RAND_MAX % n;
    int x;
    do{
        x = rand();
    }while (x >= RAND_MAX - ost);
    return mini + x % n;
}

bool SijekuSe(std::pair<vec3,vec3> s1, std::pair<vec3,vec3> s2)
{
    float a = s1.second.x - s1.first.x;
    float b = s2.first.x - s2.second.x;
    float c = s2.first.x - s1.first.x;
    float d = s1.second.y - s1.first.y;
    float e = s2.first.y - s2.second.y;
    float f = s2.first.y - s1.first.y;
    float delta = a*e - b*d;
    float delta1 = c*e - b*f;
    if(delta != 0)
    {
        float delta2 = a*f - c*d;
        return (delta1 >= 0 && delta2 >= 0 && delta1 <= delta && delta2 <= delta) || (delta1 <= 0 && delta2 <= 0 && delta1 >= delta && delta2 >= delta);
    }
    else if(delta1 == 0)
    {
        return ((c <= 0 || c <= b) && (c>=0 || c>=b) && (f<=0 || f<=e) && (f>=0 || f>=e))
        || ((c>0 || c>a) &&(c<0 || c>a) && (f>0 || f<d) && (f<0 || f<d))
        || ((a>=c || s1.second.x >= s2.second.x) && (a<=c || s1.second.x <= s2.second.x) &&
                (d>=f || s1.second.y >= s2.second.y) && (d<=f || s1.second.y <= s2.second.y))
                || ((b<c || s2.second.x > s1.second.x) && (b>c || s2.second.x < s1.second.x)
                && (e<f || s2.second.y > s1.second.y) && (e>f || s2.second.y < s1.second.y));
    }
    return false;
}

vec3 Presjek(std::pair<vec3,vec3> s1, std::pair<vec3,vec3> s2)
{
    float a = s1.second.x - s1.first.x;
    float b = s2.first.x - s2.second.x;
    float c = s2.first.x - s1.first.x;
    float d = s1.second.y - s1.first.y;
    float e = s2.first.y - s2.second.y;
    float f = s2.first.y - s1.first.y;
    float delta = a*e - b*d;
    float delta1 = c*e - b*f;
    if(delta != 0)
    {
        float delta2 = a*f - c*d;
        if((delta1>=0 && delta2>=0 && delta1<=delta && delta2<=delta) || (delta1<=0 && delta2<=0 && delta1>= delta && delta2>=delta))
        {
            float lambda = delta1/delta;
            return {(1-lambda)*s1.first.x+lambda*s1.second.x, (1-lambda)*s1.first.y+lambda*s1.second.y,0};
        }
    }
    else if(delta1 == 0)
    {
        if((c<=0 || c<=b) && (c>=0 || c>=b) && (f<=0 || f<=e) && (f>=0 || f>=e)) return s1.first;
        if((c>0 || c>a) && (c<0 || c>a) && (f>0 || f<d) && (f<0 || f<d)) return s2.first;
        if((a>=c || s1.second.x >= s2.second.x) && (a<=c || s1.second.x <=s2.second.x)) return s1.second;
        return s2.second;
    }
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
    else if(key == GLFW_KEY_H)
    {
        std::vector<vec3> pointshull = this->convHull();
        std::vector<unsigned int> indices;
        for(int i = 0; i < pointshull.size()-1; ++i)
        {
            indices.emplace_back(i);
            indices.emplace_back(i+1);
        }
        indices.emplace_back(pointshull.size()-1);
        indices.emplace_back(0);
        LinesDrawable* linije = new LinesDrawable("linije");
        linije->update_vertex_buffer(pointshull);
        linije->update_index_buffer(indices);
        this->add_drawable(linije);
        update_rendering();
    }
    else if(key == GLFW_KEY_R)
        {
            srand(time(NULL));
            int  brtacaka = SlucajniBroj(50, 100);
            int rasponmin = -20;
            int rasponmax = 20;

            std::vector<vec3> points;
            int m = 10, n =10;

            std::set<std::pair<double, double> > s; // da filtriramo duple tacke
            for(int i = 0; i < brtacaka; i++)
            {
                double x = SlucajniBroj(rasponmin, rasponmax);
                double y = SlucajniBroj(rasponmin, rasponmax);
                s.insert({x,y});
            }

            for(auto x:s)
            {
                points.emplace_back((float)x.first, (float)x.second, 0);
            }
            std::vector<vec3> tackeRect;
            int x1 = SlucajniBroj(-10, 10);
            int x2 = SlucajniBroj(-10, 10);
            while(x2 == x1) x2 = SlucajniBroj(-10,10);

            int y1 = SlucajniBroj(-10,10);
            int y2 = SlucajniBroj(-10, 10);
            while(y2 == y1) y2 = SlucajniBroj(-10,10);
            if(x1 > x2) std::swap(x1,x2);
            if(y1 > y2) std::swap(y1,y2);
            tackeRect.emplace_back(x1,y1,0);
            tackeRect.emplace_back(x2,y1,0);
            tackeRect.emplace_back(x2,y2,0);
            tackeRect.emplace_back(x1,y2,0);
            std::vector<unsigned int> indicesRect{0,1,1,2,2,3,3,0};
            pts = points;
            points = areaSearchNaive(x1,x2,y1,y2);
            PointsDrawable* tacke = new PointsDrawable("tacke");
            LinesDrawable* linije = new LinesDrawable("linije");
            PointsDrawable* tacke2 = new PointsDrawable("tacke2");
            tacke->update_vertex_buffer(pts);
            tacke2->update_vertex_buffer(points);
            linije->update_vertex_buffer(tackeRect);
            linije->update_index_buffer(indicesRect);
            this->add_drawable(tacke);
            this->add_drawable(tacke2);
            this->add_drawable(linije);
            tacke->set_default_color(vec3(1.0f, 0.0f, 0.0f));	// red color
            tacke->set_point_size(5.0f);
            tacke->set_impostor_type(PointsDrawable::SPHERE);

            tacke2->set_default_color(vec3(0.0f, 1.0f, 0.0f));	// green color
            tacke2->set_point_size(8.0f); // malo vece od obicnih tacaka
            tacke2->set_impostor_type(PointsDrawable::SPHERE);
            this->fit_screen();
            update_rendering();
        }
    else if(key == GLFW_KEY_D)
        {
            // generisi duzi
            int n = SlucajniBroj(1, 15); // h
            int m = SlucajniBroj(1, 20); // v
            std::vector<std::pair<vec3,vec3>> duzi;
            std::vector<vec3> tackeduzi;
            std::vector<uint32_t> ind;
            int j = 0;
            for(int i = 0; i < n; ++i)
            {
                int x1 = SlucajniBroj(-10, 10);
                int y1 = SlucajniBroj(-10,10);
                int x2 = SlucajniBroj(-10,10);
                int y2 = y1;
                duzi.push_back({{(float)x1,(float)y1,0},{(float)x2,(float)y2,0}});
                tackeduzi.push_back({(float)x1,(float)y1,0});
                tackeduzi.push_back({(float)x2,(float)y2,0});
                ind.push_back(j);
                ind.push_back(j+1);
                j+=2;
            }
            for(int i = 0; i < m; ++i)
            {
                int x1 = SlucajniBroj(-10, 10);
                int y1 = SlucajniBroj(-10,10);
                int x2 = x1;
                int y2 = SlucajniBroj(-10,10);
                duzi.push_back({{(float)x1,(float)y1,0},{(float)x2,(float)y2,0}});
                tackeduzi.push_back({(float)x1,(float)y1,0});
                tackeduzi.push_back({(float)x2,(float)y2,0});
                ind.push_back(j);
                ind.push_back(j+1);
                j+=2;
            }
            std::vector<vec3> P;
            for(int i = 0; i < n+m; ++i)
            {
                for(int j = i+1; j < n+m; ++j)
                    if(SijekuSe(duzi[i],duzi[j])) P.push_back(Presjek(duzi[i],duzi[j]));
            }
            PointsDrawable* tackep = new PointsDrawable("tacke");
            // crtanje
            tackep->update_vertex_buffer(P);
            this->add_drawable(tackep);
            tackep->set_default_color(vec3(1.0f, 0.0f, 0.0f));	// red color
            tackep->set_point_size(5.0f);
            tackep->set_impostor_type(PointsDrawable::SPHERE);

            LinesDrawable* duzi2 = new LinesDrawable("duzi");
            duzi2->update_vertex_buffer(tackeduzi);
            duzi2->update_index_buffer(ind);
            this->add_drawable(duzi2);

            this->fit_screen();
            update_rendering();
        }
    else if(key == GLFW_KEY_L)
    {
        // generisi duzi
        int n = SlucajniBroj(1, 15); // h
        int m = SlucajniBroj(1, 20); // v
        std::vector<std::pair<vec3,vec3>> duzi;
        std::vector<vec3> tackeduzi;
        std::vector<uint32_t> ind;
        int j = 0;
        for(int i = 0; i < n; ++i)
        {
            int x1 = SlucajniBroj(-10, 10);
            int y1 = SlucajniBroj(-10,10);
            int x2 = SlucajniBroj(-10,10);
            int y2 = SlucajniBroj(-10,10);
            duzi.push_back({{(float)x1,(float)y1,0},{(float)x2,(float)y2,0}});
            tackeduzi.push_back({(float)x1,(float)y1,0});
            tackeduzi.push_back({(float)x2,(float)y2,0});
            ind.push_back(j);
            ind.push_back(j+1);
            j+=2;
        }
        for(int i = 0; i < m; ++i)
        {
            int x1 = SlucajniBroj(-10, 10);
            int y1 = SlucajniBroj(-10,10);
            int x2 = SlucajniBroj(-10,10);
            int y2 = SlucajniBroj(-10,10);
            duzi.push_back({{(float)x1,(float)y1,0},{(float)x2,(float)y2,0}});
            tackeduzi.push_back({(float)x1,(float)y1,0});
            tackeduzi.push_back({(float)x2,(float)y2,0});
            ind.push_back(j);
            ind.push_back(j+1);
            j+=2;
        }
        std::vector<vec3> P;
        for(int i = 0; i < n+m; ++i)
        {
            for(int j = i+1; j < n+m; ++j)
                if(SijekuSe(duzi[i],duzi[j])) P.push_back(Presjek(duzi[i],duzi[j]));
        }
        PointsDrawable* tackep = new PointsDrawable("tacke");
        // crtanje
        tackep->update_vertex_buffer(P);
        this->add_drawable(tackep);
        tackep->set_default_color(vec3(1.0f, 0.0f, 0.0f));	// red color
        tackep->set_point_size(5.0f);
        tackep->set_impostor_type(PointsDrawable::SPHERE);

        LinesDrawable* duzi2 = new LinesDrawable("duzi");
        duzi2->update_vertex_buffer(tackeduzi);
        duzi2->update_index_buffer(ind);
        this->add_drawable(duzi2);

        this->fit_screen();
        update_rendering();
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


