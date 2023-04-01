bool ifslow_map1(double robotx,double roboty,double stagex,double stagey)
{   //判断是否满足减速到零的(抵达工具台)条件
    double Ax=stagex-robotx;
    double Bx=stagey-roboty;
    double distance=sqrt(Ax*Ax+Bx*Bx);
    if(robotx<2||roboty<2||robotx>48||roboty>48)
    {
        return distance<0.6;
    }
    else
    {
        return distance<0.4;
    }
}

void prevent_crash_wall_get(read_by_frame& f,int robotid)
{
    if(f.robot[robotid].x<2.0&&(f.robot[robotid].forward<(-pi/2)||f.robot[robotid].forward>(pi/2)))
    {
        printf("forward %d %f\n", robotid, 4.0);
    }
    else if(f.robot[robotid].y<2.0&&f.robot[robotid].forward<0.0)
    {
        printf("forward %d %f\n", robotid, 4.0);
    }
    else if(f.robot[robotid].x>48.0&&f.robot[robotid].forward>(-pi/2)&&f.robot[robotid].forward<(pi/2))
    {
        printf("forward %d %f\n", robotid, 4.0);
    }
    else if(f.robot[robotid].y>48.0&&f.robot[robotid].forward>0.0)
    {
        printf("forward %d %f\n", robotid, 4.0);
    }
}

void prevent_crash_wall_post(read_by_frame& f,int robotid)
{
    if(f.robot[robotid].x<1.5&&(f.robot[robotid].forward<(-pi/2)||f.robot[robotid].forward>(pi/2)))
    {
        printf("forward %d %f\n", robotid, 0.0);
    }
    else if(f.robot[robotid].y<1.5&&f.robot[robotid].forward<0.0)
    {
        printf("forward %d %f\n", robotid, 0.0);
    }
    else if(f.robot[robotid].x>48.5&&f.robot[robotid].forward>(-pi/2)&&f.robot[robotid].forward<(pi/2))
    {
        printf("forward %d %f\n", robotid, 0.0);
    }
    else if(f.robot[robotid].y>48.5&&f.robot[robotid].forward>0.0)
    {
        printf("forward %d %f\n", robotid, 0.0);
    }
}

bool move_toward_stage1_map1(read_by_frame &f,int robotID,int stageID)
{//机器人驶向某一个工作台的导航函数(第一阶段，更快)
    double angle=round_angle(f.robot[robotID].x, f.robot[robotID].y, f.stage[stageID].x, f.stage[stageID].y, f.robot[robotID].forward);
    if (fabs(angle)<0.1)
    {
        printf("rotate %d %f\n", robotID, 0.0);
        printf("forward %d %f\n",robotID,6.0);
        prevent_crash_wall_post(f,robotID);
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
                prevent_crash_wall_post(f,robotID);
                judge_crash(f, robotID);   
            }
            else
            {
                printf("forward %d %f\n",robotID, 3.0);
                printf("rotate %d %f\n", robotID, pi);
                prevent_crash_wall_post(f,robotID);
                judge_crash(f, robotID);   
            }           
        }
        else
        {
            if(fabs(angle)>pi)
            {
                printf("forward %d %f\n",robotID, 3.0);
                printf("rotate %d %f\n", robotID, pi);
                prevent_crash_wall_post(f,robotID);
                judge_crash(f, robotID);   
            }
            else
            {
                printf("forward %d %f\n",robotID, 3.0);
                printf("rotate %d %f\n", robotID, -pi);
                prevent_crash_wall_post(f,robotID);
                judge_crash(f, robotID);   
            }  
        }
        return false;
    }
}

bool move_toward_stage2_map1(read_by_frame &f,int robotID,int stageID)
{//机器人驶向某一个工作台的导航函数(第二阶段，更精细)
    double angle=round_angle(f.robot[robotID].x, f.robot[robotID].y, f.stage[stageID].x, f.stage[stageID].y, f.robot[robotID].forward);
    if(ifslow_map1(f.robot[robotID].x,f.robot[robotID].y,f.stage[stageID].x,f.stage[stageID].y))
    {
        printf("forward %d %f\n",robotID, 0.0);
        printf("rotate %d %f\n", robotID, 0.0);
        return true;
    }
    if (fabs(angle)<0.1)
    {
        printf("rotate %d %f\n", robotID, 0.0);
        printf("forward %d %f\n",robotID, 6.0);
        prevent_crash_wall_get(f,robotID);
        judge_crash(f, robotID);   
        return false;
    }
    else
    {
        if(angle>=0)
        {
            if(fabs(angle)>pi)
            {
                printf("forward %d %f\n",robotID, 2.0);
                printf("rotate %d %f\n", robotID, -pi);
                prevent_crash_wall_get(f,robotID);
                judge_crash(f, robotID);   
            }
            else
            {
                printf("forward %d %f\n",robotID, 2.0);
                printf("rotate %d %f\n", robotID, pi);
                prevent_crash_wall_get(f,robotID);
                judge_crash(f, robotID);   
            }           
        }
        else
        {
            if(fabs(angle)>pi)
            {
                printf("forward %d %f\n",robotID, 2.0);
                printf("rotate %d %f\n", robotID, pi);
                prevent_crash_wall_get(f,robotID);
                judge_crash(f, robotID);   
            }
            else
            {
                printf("forward %d %f\n",robotID, 2.0);
                printf("rotate %d %f\n", robotID, -pi);
                prevent_crash_wall_get(f,robotID);
                judge_crash(f, robotID);   
            }  
        }
        return false;
    }
}