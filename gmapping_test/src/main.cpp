#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <iostream>
#include <fstream>

typedef struct Vector2i
{
    int x;
    int y;
}Vector2i;

typedef struct Nodo
{
    int x;
    int y;
    float f = 0;
    float g = 0;
    float h = 0;
    Nodo *anterior = NULL;
}Nodo;

class PathPlanner
{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
    public:
        PathPlanner();
        void MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &grid);
        void publicar();
};

PathPlanner::PathPlanner()
{
    this->sub = nh.subscribe("map", 10, &PathPlanner::MapCallBack, this);
    this->pub = nh.advertise<nav_msgs::Path>("path", 1);
}

void PathPlanner::MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &grid)
{
    int i, j;
    std::ofstream f("mapa", std::ios::binary);
    if(f)
    {
        f.write((char*)&grid->info.width, sizeof(uint32_t));
        f.write((char*)&grid->info.height, sizeof(uint32_t));
        for(i = 0; i < grid->data.size(); i++)
        {
            f.write((char*)&grid->data.at(i), sizeof(int8_t));
        }
        f.close();
        std::cout << 1 << std::endl;
        ros::shutdown();
    }else std::cout << "no se pudo abrir el archivo" << std::endl;
    std::cout << "Res: " << std::endl;
    std::cout << grid->info.resolution << std::endl;
}

int main(int argc, char **argv)
{
    int i;

    ros::init(argc, argv, "gmapping_test");
    PathPlanner pp;

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

float mapear(float val, float valMin, float valMax, float outMin, float outMax)
{
    return (val - valMin)*(outMax-outMin)/(valMax-valMin) + outMin;
}

float distPuntos(float x1, float y1, float z1, float x2, float y2, float z2)
{
    return sqrtf((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
}

std::list<Vector2i> getCamino(Vector2i inicio, Vector2i fin, nav_msgs::OccupancyGrid grid)
{
    std::list<Nodo*> abiertos, cerrados, vecinos;   //abiertos: lista de nodos candidatos para evaluación; cerrados: lista de nodos evaluados; vecinos: lista de nodos vecinos del punto a evaluar
    std::list<Nodo*>::iterator it, it2, it3, mejor;
    std::list<Vector2i> path;
    Nodo *puntoInicial, *puntoFinal, *puntoActual, *puntoAux;
    bool flagCerrado, flagAbierto, pathNuevo;
    int i, j, progreso = 0, progresoAnterior = 0;
    float tempG, distInicial;

    //Aloja la misma cantidad de memoria que ocupa el mapa para los nodos
    Nodo **nodos = new Nodo*[grid.info.width*grid.info.height];
    for(i = 0; i < grid.info.width; i++)
        for(j = 0; j < grid.info.height; j++)
        {
            nodos[i+grid.info.width*j] = new Nodo;
            nodos[i+grid.info.width*j]->x = i;
            nodos[i+grid.info.width*j]->y = j;
        }

    puntoInicial = nodos[inicio.x+grid.info.width*inicio.y];
    puntoFinal = nodos[fin.x+grid.info.width*fin.y];
    puntoActual = puntoInicial; //Lo inicializo para que no rompa las bolas
    distInicial = distPuntos(puntoInicial->x,puntoInicial->y,0.0f,puntoFinal->x,puntoFinal->y,0.0f);

    abiertos.push_back(puntoInicial);
    if(grid.data[puntoFinal->x+grid.info.width*puntoFinal->y] == 0) //Si el punto objetivo no está bloqueado
    {
        while(!abiertos.empty())
        {
            //Elije la mejor opción de los nodos abiertos buscando el que tenga el menor valor de f
            mejor = abiertos.begin();
            for (it = abiertos.begin(); it != abiertos.end(); it++)
                if((*it)->f < (*mejor)->f) mejor = it;
            puntoActual = (*mejor);

            //Si llegó al punto final, encontró el camino y sale del bucle
            if(puntoActual == puntoFinal)
            {
                std::cout << "Camino listorti" << std::endl;
                break;
            }

            //Saca el punto que esta evaluando de la lista de nodos abiertos y la pasa a la de nodos cerrados
            abiertos.remove(puntoActual);
            cerrados.push_back(puntoActual);

            //Mete los nodos vecinos del punto a evaluar en una lista para recorrerlos más adelante
            vecinos.clear();
            for(i = -1; i < 2; i++)
                for(j = -1; j < 2; j++)
                    if(!((i==0)&&(j==0)))
                    {
                        puntoAux = nodos[(puntoActual->x+i)+grid.info.width*(puntoActual->y+j)];
                        vecinos.push_back(puntoAux);
                    }

            //Si los vecinos son candidatos a evaluación y se pueden recorrer, se calculan los factores
            //y si se trata de un camino mejor que el que se venía considerando, se pone como sucesor del punto actual
            for (it = vecinos.begin(); it != vecinos.end(); it++)
            {
                flagCerrado = false;
                flagAbierto = false;
                for (it2 = cerrados.begin(); it2 != cerrados.end(); it2++)
                    if(*it == *it2) flagCerrado = true;
                for (it3 = abiertos.begin(); it3 != abiertos.end(); it3++)
                    if(*it == *it3) flagAbierto = true;
                if((!flagCerrado) && (grid.data[(*it)->x+grid.info.width*(*it)->y] == 0))
                {
                    tempG = puntoActual->g + distPuntos((*it)->x,(*it)->y,0.0f,puntoActual->x,puntoActual->y,0.0f);
                    pathNuevo = false;
                    if(flagAbierto)
                    {
                        if(tempG < (*it)->g)
                        {
                            (*it)->g = tempG;
                            pathNuevo = true;
                        }
                    }else{
                        (*it)->g = tempG;
                        pathNuevo = true;
                        abiertos.push_back(*it);
                    }

                    if(pathNuevo)
                    {
                        (*it)->h = distPuntos((*it)->x,(*it)->y,0.0f,puntoFinal->x,puntoFinal->y,0.0f);
                        (*it)->f = (*it)->g + (*it)->h;
                        (*it)->anterior = puntoActual;
                    }
                }
            }
        }
    }else //Si el objetivo está bloqueado función termina devolviendo el punto final
    {
        path.push_back((Vector2i){puntoFinal->x,puntoFinal->y});
        return path;
    }

    //Reconstruye el camino y lo guarda en una lista de vectores de dos dimensiones
    puntoAux = puntoActual;
    path.push_front((Vector2i){puntoAux->x, puntoAux->y});
    while(puntoAux->anterior != NULL)
    {
        path.push_front((Vector2i){puntoAux->anterior->x, puntoAux->anterior->y});
        puntoAux = puntoAux->anterior;
    }

    //Borra la matriz de punteros a nodos
    for(i = 0; i < grid.info.width*grid.info.height; ++i)
        delete [] nodos[i];

    return path;
}