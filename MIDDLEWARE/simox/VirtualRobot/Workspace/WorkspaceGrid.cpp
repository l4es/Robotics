
#include "WorkspaceGrid.h"
#include "../VirtualRobotException.h"
#include "../ManipulationObject.h"
#include <iostream>
#include <algorithm>
using namespace std;

#define MIN_VALUES_STORE_GRASPS 100

namespace VirtualRobot
{

    WorkspaceGrid::WorkspaceGrid(float fMinX, float fMaxX, float fMinY, float fMaxY, float fDiscretizeSize)
    {

        minX = fMinX;
        maxX = fMaxX;
        minY = fMinY;
        maxY = fMaxY;
        discretizeSize = fDiscretizeSize;
        data = NULL;

        if (fMinX >= fMaxX || fMinY >= fMaxY)
        {
            THROW_VR_EXCEPTION("ERROR min >= max");
        }

        gridExtendX = maxX - minX;
        gridExtendY = maxY - minY;
        gridSizeX = (int)((maxX - minX) / discretizeSize) + 1;
        gridSizeY = (int)((maxY - minY) / discretizeSize) + 1;


        if (gridSizeY <= 0 || gridSizeX <= 0 || gridExtendX <= 0 || gridExtendY <= 0)
        {
            THROW_VR_EXCEPTION("ERROR negative grid size");
        }

        VR_INFO << ": creating grid with " << gridSizeX << "x" << gridSizeY << " = " << gridSizeX* gridSizeY << " entries" << endl;
        data = new int[gridSizeX * gridSizeY];
        graspLink = new std::vector<GraspPtr>[gridSizeX * gridSizeY];
        memset(data, 0, sizeof(int)*gridSizeX * gridSizeY);
    }

    WorkspaceGrid::~WorkspaceGrid()
    {
        delete []data;
        delete []graspLink;
    }

    void WorkspaceGrid::reset()
    {
        delete []graspLink;
        graspLink = new std::vector<GraspPtr>[gridSizeX * gridSizeY];
        memset(data, 0, sizeof(int)*gridSizeX * gridSizeY);
    }

    int WorkspaceGrid::getEntry(float x, float y)
    {
        if (!data)
        {
            return 0;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);

        if (nPosX < 0 || nPosY < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << endl;
            return 0;
        }

        return data[getDataPos(nPosX, nPosY)];
    }

    bool WorkspaceGrid::getEntry(float x, float y, int& storeEntry, GraspPtr& storeGrasp)
    {
        if (!data)
        {
            return false;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);

        if (nPosX < 0 || nPosY < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << endl;
            return false;
        }

        storeEntry = data[getDataPos(nPosX, nPosY)];
        int nLinks = graspLink[getDataPos(nPosX, nPosY)].size();

        if (nLinks > 0)
        {
            storeGrasp = graspLink[getDataPos(nPosX, nPosY)][rand() % nLinks];    // get random grasp
        }
        else
        {
            storeGrasp.reset();
        }

        return true;
    }

    bool WorkspaceGrid::getEntry(float x, float y, int& storeEntry, std::vector<GraspPtr>& storeGrasps)
    {
        if (!data)
        {
            return false;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);

        if (nPosX < 0 || nPosY < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << endl;
            return false;
        }

        storeEntry = data[getDataPos(nPosX, nPosY)];
        storeGrasps = graspLink[getDataPos(nPosX, nPosY)];
        return true;
    }

    int WorkspaceGrid::getCellEntry(int cellX, int cellY)
    {
        if (!data)
        {
            return 0;
        }

        if (cellX < 0 || cellY < 0 || cellX >= gridSizeX || cellY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << endl;
            return 0;
        }

        return data[getDataPos(cellX, cellY)];
    }

    bool WorkspaceGrid::getCellEntry(int cellX, int cellY, int& storeEntry, GraspPtr& storeGrasp)
    {
        if (!data)
        {
            return false;
        }

        if (cellX < 0 || cellY < 0 || cellX >= gridSizeX || cellY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << endl;
            return false;
        }

        storeEntry = data[getDataPos(cellX, cellY)];
        int nLinks = graspLink[getDataPos(cellX, cellY)].size();

        if (nLinks > 0)
        {
            storeGrasp = graspLink[getDataPos(cellX, cellY)][rand() % nLinks];
        }
        else
        {
            storeGrasp.reset();
        }

        return true;
    }

    bool WorkspaceGrid::getCellEntry(int cellX, int cellY, int& storeEntry, std::vector<GraspPtr>& storeGrasps)
    {
        if (!data)
        {
            return false;
        }

        if (cellX < 0 || cellY < 0 || cellX >= gridSizeX || cellY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << endl;
            return false;
        }

        storeEntry = data[getDataPos(cellX, cellY)];
        storeGrasps = graspLink[getDataPos(cellX, cellY)];
        return true;
    }

    void WorkspaceGrid::setEntry(float x, float y, int value, GraspPtr grasp)
    {
        if (!data)
        {
            return;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);
        setCellEntry(nPosX, nPosY, value, grasp);
    }

    void WorkspaceGrid::setCellEntry(int cellX, int cellY, int value, GraspPtr grasp)
    {
        if (!data)
        {
            return;
        }

        if (cellX < 0 || cellY < 0 || cellX >= gridSizeX || cellY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nPosX << "," << nPosY << endl;
            return;
        }

        if (data[getDataPos(cellX, cellY)] <= value)
        {
            data[getDataPos(cellX, cellY)] = value;

            if (find(graspLink[getDataPos(cellX, cellY)].begin(), graspLink[getDataPos(cellX, cellY)].end(), grasp) == graspLink[getDataPos(cellX, cellY)].end())
            {
                graspLink[getDataPos(cellX, cellY)].push_back(grasp);
            }
        }
        else if (value >= MIN_VALUES_STORE_GRASPS)
        {
            if (find(graspLink[getDataPos(cellX, cellY)].begin(), graspLink[getDataPos(cellX, cellY)].end(), grasp) == graspLink[getDataPos(cellX, cellY)].end())
            {
                graspLink[getDataPos(cellX, cellY)].push_back(grasp);
            }
        }
    }

    void WorkspaceGrid::setEntryCheckNeighbors(float x, float y, int value, GraspPtr grasp)
    {
        if (!data)
        {
            return;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);

        if (nPosX < 0 || nPosY < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << endl;
            return;
        }

        setCellEntry(nPosX, nPosY, value, grasp);

        if (nPosX > 0 && nPosX < (gridSizeX - 1) && nPosY > 0 && nPosY < (gridSizeY - 1))
        {
            setCellEntry(nPosX - 1, nPosY, value, grasp);
            setCellEntry(nPosX - 1, nPosY - 1, value, grasp);
            setCellEntry(nPosX - 1, nPosY + 1, value, grasp);
            setCellEntry(nPosX, nPosY - 1, value, grasp);
            setCellEntry(nPosX, nPosY + 1, value, grasp);
            setCellEntry(nPosX + 1, nPosY - 1, value, grasp);
            setCellEntry(nPosX + 1, nPosY, value, grasp);
            setCellEntry(nPosX + 1, nPosY + 1, value, grasp);
        }
    }

    void WorkspaceGrid::setEntries(std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr>& wsData, Eigen::Matrix4f& graspGlobal, GraspPtr grasp)
    {
        if (!data)
        {
            return;
        }

        float x, y;

        for (int i = 0; i < (int)wsData.size(); i++)
        {
            Eigen::Matrix4f tmpPos2 = graspGlobal * wsData[i]->transformation.inverse();
            x = tmpPos2(0, 3);
            y = tmpPos2(1, 3);

            setEntryCheckNeighbors(x, y, wsData[i]->value, grasp);
            //setEntry(x,y,vData[i].value);
        }
    }

    int WorkspaceGrid::getMaxEntry()
    {
        if (!data)
        {
            return 0;
        }

        int nMax = 0;

        for (int i = 0; i < gridSizeX; i++)
            for (int j = 0; j < gridSizeY; j++)
                if (data[getDataPos(i, j)] > nMax)
                {
                    nMax = data[getDataPos(i, j)];
                }

        return nMax;
    }
    /*
    void WorkspaceGrid::visualize(SoSeparator *pStoreResult)
    {
        if (!data || !pStoreResult)
            return;
        float sizeX,sizeY;
        sizeX = gridExtendX / (float)gridSizeX;
        sizeY = gridExtendY / (float)gridSizeY;

        / *SoCube *cube = new SoCube();
        cube->width = sizeX*1.1f;
        cube->height = sizeY*1.1f;
        cube->depth = 1.0f;* /
        SoSphere *cube = new SoSphere();
        cube->radius.setValue(sizeX*1.0f);
        float x,y;

        int nMaxValue = getMaxEntry();
        if (nMaxValue==0)
            nMaxValue = 1;
        int value;

        for (int i=0;i<gridSizeX;i++)
        {
            for (int j=0;j<gridSizeY;j++)
            {
                if (data[getDataPos(i,j)]>0)
                {
                    value = data[getDataPos(i,j)];

                    SoSeparator *pSep1 = new SoSeparator();
                    x = minX + ((float)i+0.5f)*discretizeSize;
                    y = minY + ((float)j+0.5f)*discretizeSize;
                    SoTranslation *tr = new SoTranslation();
                    tr->translation.setValue(x,y,0);
                    float mat1 = (float)value/(float)nMaxValue;
                    mat1 *= 1.0f;
                    if (mat1<0.0f)
                        mat1 = 0.0f;
                    if (mat1>0.8f)
                        mat1 = 0.8f;
                    mat1 = 1.0f-mat1;
                    SoMaterial *mat= new SoMaterial();
                    mat->diffuseColor.setValue(1.0f,mat1,mat1);
                    mat->ambientColor.setValue(1.0f,mat1,mat1);

                    pSep1->addChild(tr);
                    pSep1->addChild(mat);
                    pSep1->addChild(cube);

                    pStoreResult->addChild(pSep1);
                }
            }
        }
    }*/

    bool WorkspaceGrid::getRandomPos(int minEntry, float& storeXGlobal, float& storeYGlobal, GraspPtr& storeGrasp, int maxLoops /*= 50*/)
    {
        if (!data)
        {
            return false;
        }

        int nLoop = 0;
        int nEntry = 0;
        int x, y;

        do
        {
            x = rand() % gridSizeX;
            y = rand() % gridSizeY;
            getCellEntry(x, y, nEntry, storeGrasp);

            if (nEntry >= minEntry)
            {
                storeXGlobal = minX + ((float)x + 0.5f) * discretizeSize;
                storeYGlobal = minY + ((float)y + 0.5f) * discretizeSize;
                return true;
            }

            nLoop++;
        }
        while (nLoop < maxLoops);

        return false;
    }


    bool WorkspaceGrid::getRandomPos(int minEntry, float& storeXGlobal, float& storeYGlobal, std::vector<GraspPtr>& storeGrasps, int maxLoops /*= 50*/)
    {
        if (!data)
        {
            return false;
        }

        int nLoop = 0;
        int nEntry = 0;
        int x, y;

        do
        {
            x = rand() % gridSizeX;
            y = rand() % gridSizeY;
            getCellEntry(x, y, nEntry, storeGrasps);

            if (nEntry >= minEntry)
            {
                storeXGlobal = minX + ((float)x + 0.5f) * discretizeSize;
                storeYGlobal = minY + ((float)y + 0.5f) * discretizeSize;
                return true;
            }

            nLoop++;
        }
        while (nLoop < maxLoops);

        return false;
    }

    void WorkspaceGrid::setGridPosition(float x, float y)
    {
        minX = x - gridExtendX / 2.0f;
        maxX = x + gridExtendX / 2.0f;
        minY = y - gridExtendY / 2.0f;
        maxY = y + gridExtendY / 2.0f;
    }

    bool WorkspaceGrid::fillGridData(WorkspaceRepresentationPtr ws, ManipulationObjectPtr o, GraspPtr g, RobotNodePtr baseRobotNode)
    {
        if (!ws || !o || !g)
        {
            return false;
        }

        Eigen::Matrix4f graspGlobal = g->getTcpPoseGlobal(o->getGlobalPose());

        WorkspaceRepresentation::WorkspaceCut2DPtr cutXY = ws->createCut(graspGlobal, discretizeSize);

        std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> transformations = ws->createCutTransformations(cutXY, baseRobotNode);
        setEntries(transformations, graspGlobal, g);
        return true;
    }

    void WorkspaceGrid::getExtends(float& storeMinX, float& storeMaxX, float& storeMinY, float& storeMaxY)
    {
        storeMinX = minX;
        storeMaxX = maxX;
        storeMinY = minY;
        storeMaxY = maxY;
    }

    void WorkspaceGrid::getCells(int& storeCellsX, int& storeCellsY)
    {
        storeCellsX = gridSizeX;
        storeCellsY = gridSizeY;
    }

} //  namespace

