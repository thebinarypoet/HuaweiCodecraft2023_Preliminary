bool ifslow_map3(double robotx,double roboty,double stagex,double stagey)
{   //判断是否满足减速到零的(抵达工具台)条件
    double Ax=stagex-robotx;
    double Bx=stagey-roboty;
    double distance=sqrt(Ax*Ax+Bx*Bx);
    return distance<0.4;
}

bool move_toward_stage1_map3(read_by_frame &f,int robotID,int stageID)
{//机器人驶向某一个工作台的导航函数(第一阶段，更快)
    double angle=round_angle(f.robot[robotID].x, f.robot[robotID].y, f.stage[stageID].x, f.stage[stageID].y, f.robot[robotID].forward);
    if (fabs(angle)<0.1)
    {
        printf("rotate %d %f\n", robotID, 0.0);
        printf("forward %d %f\n",robotID,6.0);
        judge_crash(f, robotID);   
        return false;
    }
    else
    {
        if(angle>=0)
        {
            if(fabs(angle)>pi)
            {
                printf("forward %d %f\n",robotID, 3.0);
                printf("rotate %d %f\n", robotID, -pi);
                judge_crash(f, robotID);   
            }
            else
            {
                printf("forward %d %f\n",robotID, 3.0);
                printf("rotate %d %f\n", robotID, pi);
                judge_crash(f, robotID);   
            }           
        }
        else
        {
            if(fabs(angle)>pi)
            {
                printf("forward %d %f\n",robotID, 3.0);
                printf("rotate %d %f\n", robotID, pi);
                judge_crash(f, robotID);   
            }
            else
            {
                printf("forward %d %f\n",robotID, 3.0);
                printf("rotate %d %f\n", robotID, -pi);
                judge_crash(f, robotID);   
            }  
        }
        return false;
    }
}

bool move_toward_stage2_map3(read_by_frame &f,int robotID,int stageID)
{//机器人驶向某一个工作台的导航函数(第二阶段，更精细)
    double angle=round_angle(f.robot[robotID].x, f.robot[robotID].y, f.stage[stageID].x, f.stage[stageID].y, f.robot[robotID].forward);
    if(ifslow_map3(f.robot[robotID].x,f.robot[robotID].y,f.stage[stageID].x,f.stage[stageID].y))
    {
        printf("forward %d %f\n",robotID, 0.0);
        printf("rotate %d %f\n", robotID, 0.0);
        return true;
    }
    if (fabs(angle)<0.1)
    {
        printf("rotate %d %f\n", robotID, 0.0);
        printf("forward %d %f\n",robotID, 6.0);
        judge_crash(f, robotID);   
        return false;
    }
    else
    {
        if(angle>=0)
        {
            if(fabs(angle)>pi)
            {
                printf("forward %d %f\n",robotID, 2.1);
                printf("rotate %d %f\n", robotID, -pi);
                judge_crash(f, robotID);   
            }
            else
            {
                printf("forward %d %f\n",robotID, 2.1);
                printf("rotate %d %f\n", robotID, pi);
                judge_crash(f, robotID);   
            }           
        }
        else
        {
            if(fabs(angle)>pi)
            {
                printf("forward %d %f\n",robotID, 2.1);
                printf("rotate %d %f\n", robotID, pi);
                judge_crash(f, robotID);   
            }
            else
            {
                printf("forward %d %f\n",robotID, 2.1);
                printf("rotate %d %f\n", robotID, -pi);
                judge_crash(f, robotID);   
            }  
        }
        if(frameID > 4197 && stagingNum == 18 && frameID < 4300 && f.robot[robotID].productType == 6)
            printf("forward %d %f\n",robotID, 1.0);
        return false;
    }
}
