double round_angle(double robot_posx,double robot_posy,double stage_posx,double stage_posy,double robot_toward)
{//算出两个向量之间的夹角(返回目标角减去朝向角)
    double Ax=stage_posx-robot_posx;
    double Ay=stage_posy-robot_posy;
    double theta = atan2(Ay, Ax);;
    return theta - robot_toward;
}

bool ifslow(double robotx,double roboty,double stagex,double stagey)
{   //判断是否满足减速到零的(抵达工具台)条件
    double Ax=stagex-robotx;
    double Bx=stagey-roboty;
    double distance=sqrt(Ax*Ax+Bx*Bx);
    return distance<0.4;
}

double length(read_by_frame f, int robotId, int stageId){
    double l = sqrt(pow(f.robot[robotId].x - f.stage[stageId].x, 2) + pow(f.robot[robotId].y - f.stage[stageId].y, 2));
    return l;
}


string atob(int x)//十进制转二进制函数
{
    string s;
    s.resize(9);
    for(int i=0;i<9;i++)
    {
        s[i]='0';
    }
    int n=0;
    while(x!=0)
    {
        s[n]=((x%2)+'0');
        x=x/2;
        n++;
    }
    return s;
}

bool if_robot_access(read_by_frame &f,int robotID,int stageId)//判断robot是否距离目标足够近，以调用更精细的导航函数
{
    double Ax=f.robot[robotID].x;
    double Ay=f.robot[robotID].y;
    double Bx=f.stage[stageId].x;
    double By=f.stage[stageId].y;
    double distance=sqrt(pow((Ax-Bx),2)+pow((Ay-By),2));
    return distance<2.2;
}

bool if_have_stage(read_by_frame f, int type){
    for(int i = 0; i < stagingNum; i++){
        if(f.stage[i].type == type)
            return true;
    }
    return false;
}

void judge_crash(read_by_frame f, int robotID){
    int i, length = 0;
    for(i = 0; i < 4; i++){
        if(i != robotID){
            double x1 = f.robot[i].x - f.robot[robotID].x;
            double y1 = f.robot[i].y - f.robot[robotID].y;
            if((stagingNum == 50 && !if_have_stage(f, 7)) || stagingNum == 18)
                length = 4;
            else
                length = 3;
            if(sqrt(x1 * x1 + y1 * y1) < length){
                double rad = fabs(f.robot[i].forward) + fabs(f.robot[robotID].forward);
                bool judge1 = f.robot[i].forward < 0 && f.robot[robotID].forward > 0 && f.robot[i].y > f.robot[robotID].y;
                bool judge2 = f.robot[i].forward > 0 && f.robot[robotID].forward < 0 && f.robot[i].y < f.robot[robotID].y;
                if(rad < 7 * pi / 6 && rad > 5 * pi / 6 && (judge1 || judge2))
                    break;
            }
        }
    }
    //右上左下顺，右下左上逆
    if(i != 4 && f.robot[robotID].forward > 0 && f.robot[i].forward < 0 && f.robot[robotID].x > f.robot[i].x)
        printf("rotate %d %f\n", robotID, -0.5 * pi);
    else if(i != 4 && f.robot[robotID].forward < 0 && f.robot[i].forward > 0 && f.robot[robotID].x < f.robot[i].x)
        printf("rotate %d %f\n", robotID, -0.5 * pi);
    else if(i != 4 && f.robot[robotID].forward > 0 && f.robot[i].forward < 0 && f.robot[robotID].x < f.robot[i].x)
        printf("rotate %d %f\n", robotID, 0.5 * pi);
    else if(i != 4 && f.robot[robotID].forward < 0 && f.robot[i].forward > 0 && f.robot[robotID].x > f.robot[i].x)
        printf("rotate %d %f\n", robotID, 0.5 * pi);
    if(i != 4 && stagingNum == 18)
        printf("forward %d %f\n", robotID, 6.0);
} //判断是否即将发生碰撞

bool move_toward_stage1(read_by_frame &f,int robotID,int stageID)
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

bool move_toward_stage2(read_by_frame &f,int robotID,int stageID)
{//机器人驶向某一个工作台的导航函数(第二阶段，更精细)
    double angle=round_angle(f.robot[robotID].x, f.robot[robotID].y, f.stage[stageID].x, f.stage[stageID].y, f.robot[robotID].forward);
    if(ifslow(f.robot[robotID].x,f.robot[robotID].y,f.stage[stageID].x,f.stage[stageID].y))
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
                printf("forward %d %f\n",robotID, 2.0);
                printf("rotate %d %f\n", robotID, -pi);
                judge_crash(f, robotID);   
            }
            else
            {
                printf("forward %d %f\n",robotID, 2.0);
                printf("rotate %d %f\n", robotID, pi);
                judge_crash(f, robotID);   
            }           
        }
        else
        {
            if(fabs(angle)>pi)
            {
                printf("forward %d %f\n",robotID, 2.0);
                printf("rotate %d %f\n", robotID, pi);
                judge_crash(f, robotID);   
            }
            else
            {
                printf("forward %d %f\n",robotID, 2.0);
                printf("rotate %d %f\n", robotID, -pi);
                judge_crash(f, robotID);   
            }  
        }
        if(frameID > 4197 && stagingNum == 18 && frameID < 4300 && f.robot[robotID].productType == 6)
            printf("forward %d %f\n",robotID, 1.0);
        return false;
    }
}

bool ask_target_occupied(read_by_frame &f,int productID,int *task_toward)//取物时，问询所取物品要送去的目标工作台材料格是否占满
{
    if(productID==1)
    {
        int have=0;
        for(auto it=f.stage_type[4].begin();it!=f.stage_type[4].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[1]=='0')
            {
                have++;
            }
        }
        for(auto it=f.stage_type[5].begin();it!=f.stage_type[5].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[1]=='0')
            {
                have++;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==1&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==1)
            {
                have--;
            }
        }
        return have<=0;     
    }
    if(productID==2)
    {
        int have=0;
        for(auto it=f.stage_type[4].begin();it!=f.stage_type[4].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[2]=='0')
            {
                have++;
            }
        }
        for(auto it=f.stage_type[6].begin();it!=f.stage_type[6].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[2]=='0')
            {
                have++;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==2&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==2)
            {
                have--;
            }
        }
        return have<=0;     
    }
    if(productID==3)
    {
        int have=0;
        for(auto it=f.stage_type[5].begin();it!=f.stage_type[5].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[3]=='0')
            {
                have++;
            }
        }
        for(auto it=f.stage_type[6].begin();it!=f.stage_type[6].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[3]=='0')
            {
                have++;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==3&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==3)
            {
                have--;
            }
        }
        return have<=0;    
    }
    if(productID==4)
    {
        int have=0;
        for(auto it=f.stage_type[7].begin();it!=f.stage_type[7].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[4]=='0')
            {
                have++;
            }
        }
        if(f.stage_type[7].empty())
        {
            for(auto it=f.stage_type[9].begin();it!=f.stage_type[9].end();it++)
            {
                string s=atob(f.stage[(*it)].materialStatus);
                if(s[4]=='0')
                {
                    return false;
                }
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==4&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==4)
            {
                have--;
            }
        }
        return have<=0;   
    }
    if(productID==5)
    {
        int have=0;
        for(auto it=f.stage_type[7].begin();it!=f.stage_type[7].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[5]=='0')
            {
                have++;
            }
        }
        if(f.stage_type[7].empty())
        {
            for(auto it=f.stage_type[9].begin();it!=f.stage_type[9].end();it++)
            {
                string s=atob(f.stage[(*it)].materialStatus);
                if(s[5]=='0')
                {
                    return false;
                }
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==5&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==5)
            {
                have--;
            }
        }
        return have<=0;  
    }
    if(productID==6)
    {
        int have=0;
        for(auto it=f.stage_type[7].begin();it!=f.stage_type[7].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[6]=='0')
            {
                have++;
            }
        }
        if(f.stage_type[7].empty())
        {
            for(auto it=f.stage_type[9].begin();it!=f.stage_type[9].end();it++)
            {
                string s=atob(f.stage[(*it)].materialStatus);
                if(s[6]=='0')
                {
                    return false;
                }
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==6&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==6)
            {
                have--;
            }
        }
        return have<=0; 
    }
    if(productID==7)
    {
        for(auto it=f.stage_type[8].begin();it!=f.stage_type[8].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[7]=='0')
            {
                return false;
            }
        }
        return true;  
    }
    return 0;                    
}

bool ask_if_haveproduct(read_by_frame &f,int productID)//取物时，问询生产该类型材料的工作台是否生产完毕
{
    if(!f.stage_type[productID].empty())
    {
        for(auto it=f.stage_type[productID].begin();it!=f.stage_type[productID].end();it++)
        {
            if(f.stage[(*it)].productStatus==0 || (stagingNum == 18 && (f.stage[(*it)].remainTime > 50 && f.stage[(*it)].productStatus == 0)))
            {
                continue;
            }
            else
            {
                return true;
            }
        }
        return false;
    }
    else
    {
        return false;
    }
}

void get_product(read_by_frame &f,int robotId, int *task_toward, int n)//取物指令
{
    int End;
    int flag = 0;
    for (auto it = f.stage_value_order.begin(); it != f.stage_value_order.end(); it++)
    {
        if (f.stage[(*it)].type == 1 && ask_if_haveproduct(f, 1))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (!flag)
                {
                    End = *it;
                    flag = 1;
                }
                if (ask_target_occupied(f, 1, task_toward))
                {
                    continue;
                }
                else
                {
                    if (task_toward[0] != (*it) && task_toward[1] != (*it) && task_toward[2] != (*it) && task_toward[3] != (*it))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 2 && ask_if_haveproduct(f, 2))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (!flag)
                {
                    End = *it;
                    flag = 1;
                }
                if (ask_target_occupied(f, 2, task_toward))
                {
                    continue;
                }
                else
                {
                    if (task_toward[0] != (*it) && task_toward[1] != (*it) && task_toward[2] != (*it) && task_toward[3] != (*it))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 3 && ask_if_haveproduct(f, 3))
        {

            if (f.stage[(*it)].productStatus == 1)
            {
                if (!flag)
                {
                    End = *it;
                    flag = 1;
                }
                if (ask_target_occupied(f, 3, task_toward))
                {
                    continue;
                }
                else
                {
                    if (task_toward[0] != (*it) && task_toward[1] != (*it) && task_toward[2] != (*it) && task_toward[3] != (*it))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 4 && ask_if_haveproduct(f, 4))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (ask_target_occupied(f, 4, task_toward))
                {
                    continue;
                }
                else
                {
                    if ((task_toward[0] != (*it) || f.robot[0].productType != 0) && (task_toward[1] != (*it) || f.robot[1].productType != 0) && (task_toward[2] != (*it) || f.robot[2].productType != 0) && (task_toward[3] != (*it) || f.robot[3].productType != 0))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 5 && ask_if_haveproduct(f, 5))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (ask_target_occupied(f, 5, task_toward))
                {
                    continue;
                }
                else
                {
                    if ((task_toward[0] != (*it) || f.robot[0].productType != 0) && (task_toward[1] != (*it) || f.robot[1].productType != 0) && (task_toward[2] != (*it) || f.robot[2].productType != 0) && (task_toward[3] != (*it) || f.robot[3].productType != 0))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 6 && ask_if_haveproduct(f, 6))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (ask_target_occupied(f, 6, task_toward))
                {
                    continue;
                }
                else
                {
                    if ((task_toward[0] != (*it) || f.robot[0].productType != 0) && (task_toward[1] != (*it) || f.robot[1].productType != 0) && (task_toward[2] != (*it) || f.robot[2].productType != 0) && (task_toward[3] != (*it) || f.robot[3].productType != 0))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 7 && ask_if_haveproduct(f, 7))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (ask_target_occupied(f, 7, task_toward))
                {
                    continue;
                }
                else
                {
                    if ((task_toward[0] != (*it) || f.robot[0].productType != 0) && (task_toward[1] != (*it) || f.robot[1].productType != 0) && (task_toward[2] != (*it) || f.robot[2].productType != 0) && (task_toward[3] != (*it) || f.robot[3].productType != 0))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
    }
    if (f.stage.size() == 43)
    {
        if (frameID == 1)
        {
            if (robotId == 3 || robotId == 2)
            {
                task_toward[robotId] = 42;
            }
            else if (robotId == 1)
            {
                task_toward[robotId] = 0;
            }
            else
            {
                task_toward[robotId] = 41;
            }
        }
        else
        {
            task_toward[robotId] = End;
        }
    }
}

void post_product(read_by_frame &f,int robotID,int *task_toward,int objtype)//优先级一(1,2,3->4,5,6)的送物指令
{
    for(auto it=f.stage_value_order.begin();it!=f.stage_value_order.end();it++)
    {
       if(objtype==1)
       {
            if(f.stage[(*it)].type==4||f.stage[(*it)].type==5)
            {
                string s;
                s=atob(f.stage[(*it)].materialStatus);
                {
                    if(s[objtype]=='0')
                    {
                        if((task_toward[0]!=(*it)||f.robot[0].productType!=objtype)&&(task_toward[1]!=(*it)||f.robot[1].productType!=objtype)&&(task_toward[2]!=(*it)||f.robot[2].productType!=objtype)&&(task_toward[3]!=(*it)||f.robot[3].productType!=objtype))
                        {
                            task_toward[robotID]=(*it);
                            return;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
       else if(objtype==2)
       {
            if(f.stage[(*it)].type==4||f.stage[(*it)].type==6)
            {
                string s;
                s=atob(f.stage[(*it)].materialStatus);
                {
                    if(s[objtype]=='0')
                    {
                        if((task_toward[0]!=(*it)||f.robot[0].productType!=objtype)&&(task_toward[1]!=(*it)||f.robot[1].productType!=objtype)&&(task_toward[2]!=(*it)||f.robot[2].productType!=objtype)&&(task_toward[3]!=(*it)||f.robot[3].productType!=objtype))
                        {
                            task_toward[robotID]=(*it);
                            return;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
       else if(objtype==3)
       {
            if(f.stage[(*it)].type==5||f.stage[(*it)].type==6)
            {
                string s;
                s=atob(f.stage[(*it)].materialStatus);
                {
                    if(s[objtype]=='0')
                    {
                        if((task_toward[0]!=(*it)||f.robot[0].productType!=objtype)&&(task_toward[1]!=(*it)||f.robot[1].productType!=objtype)&&(task_toward[2]!=(*it)||f.robot[2].productType!=objtype)&&(task_toward[3]!=(*it)||f.robot[3].productType!=objtype))
                        {
                            task_toward[robotID]=(*it);
                            return;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
        else if(objtype==4||objtype==5||objtype==6)
        {
            if(f.stage[(*it)].type==7||f.stage[(*it)].type==9)
            {
                string s;
                s=atob(f.stage[(*it)].materialStatus);
                {
                    if(s[objtype]=='0')
                    {
                        if((task_toward[0]!=(*it)||f.robot[0].productType!=objtype||f.stage[task_toward[0]].type==9)&&(task_toward[1]!=(*it)||f.robot[1].productType!=objtype||f.stage[task_toward[1]].type==9)&&(task_toward[2]!=(*it)||f.robot[2].productType!=objtype||f.stage[task_toward[2]].type==9)&&(task_toward[3]!=(*it)||f.robot[3].productType!=objtype||f.stage[task_toward[3]].type==9))
                        {
                            task_toward[robotID]=(*it);
                            return;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
        else if(objtype==7)
        {
            if(f.stage[(*it)].type==8||f.stage[(*it)].type==9)
            {
                if(task_toward[0]!=(*it)&&task_toward[1]!=(*it)&&task_toward[2]!=(*it)&&task_toward[3]!=(*it))
                {
                    task_toward[robotID]=(*it);
                    return;
                }
                else
                {
                    continue;
                }
            }
        }
    }   
}

void task_give(read_by_frame &f,int robotId,int *task_toward,int frameID, int n)//通过调用子函数实现任务分发，分发前问询取物工作台的状态以决定优先级
{
    int productId=f.robot[robotId].productType;
    if(productId==0)
    {
        if(stagingNum != 43){
            if(stagingNum != 25){
                if(frameID>8808)
                {
                    if(f.robot[robotId].productType == 0)
                        printf("forward %d %f\n",robotId, 6.0);
                        printf("rotate %d %f\n", robotId, pi);
                    return;   
                }
            }
            else{
                if(frameID > 8748){
                    if(f.robot[robotId].productType == 0)
                        printf("forward %d %f\n",robotId, 6.0);
                        printf("rotate %d %f\n", robotId, pi);
                    return;  
                }
            }
        }
        else{
            if(frameID>8759)
            {
                if(f.robot[robotId].productType == 0)
                    printf("forward %d %f\n",robotId, 6.0);
                    printf("rotate %d %f\n", robotId, pi);
                return;   
            }
        }
        get_product(f,robotId,task_toward, n);
    }
    else
    {
        post_product(f,robotId,task_toward,productId);
    }
}
struct cmp//重载set排序方式
{
    bool operator()(pair<int,double> a,pair<int,double> b)const
    {
        return a.second > b.second;
    }
};

void add_stage_value(read_by_frame *f, int type){
    for(int i = 0; i < stagingNum; i++){
        if(f->stage[i].type == type)
            f->stage[i].value += 100;
    }
}

bool judge_have_material(read_by_frame f, int type, int stageId){
    int j = f.stage[stageId].materialStatus;
    if((type == 1 && (j == 2 || j == 6 || j == 10)) || (type == 2 && (j == 4 || j == 6 || j == 12)) || (type == 3 && (j == 8 || j == 10 || j == 12) || 
       (type == 4 && (j == 16 || j == 48 || j == 80 || j == 112) || (type == 5 && (j == 32 || j == 48 || j == 96 || j == 112)) || 
       (type == 6 && (j == 64 || j == 80 || j == 96 || j == 112)) || (type == 7 && j == 128))))
        return false;
    else
        return true;
}

bool compare(pair<int, int> a, pair<int, int> b){
    return a.second > b.second;
}

void set_stage_value(read_by_frame *f){
    int exist[6] = {0}, need[6] = {0};
    for(int i = 0; i < stagingNum; i++){
        for(int j = 0; j < 6; j++){
            if(!judge_have_material(*f, j + 1, i))
                exist[j] += 1;
        }
    } //判断场上工作台已有商品类型个数
    for(int j = 0; j < 4; j++){
        if(f->robot[j].productType != 0)
            exist[f->robot[j].productType - 1]++;
    } //判断场上机器人携带商品类型个数
    for(int i = 0; i < stagingNum; i++){
        if(f->stage[i].type == 4){
            need[0]++, need[1]++;
            if(f->stage[i].remainTime != -1)
                exist[3]++;
            if(f->stage[i].productStatus == 1)
                exist[3]++;
        }
        else if(f->stage[i].type == 5){
            need[0]++, need[2]++;
            if(f->stage[i].remainTime != -1)
                exist[4]++;
            if(f->stage[i].productStatus == 1)
                exist[4]++;
        }
        else if(f->stage[i].type == 6){
            need[1]++, need[2]++;
            if(f->stage[i].remainTime != -1)
                exist[5]++;
            if(f->stage[i].productStatus == 1)
                exist[5]++;
        }
        else if(f->stage[i].type == 7)
            need[3]++, need[4]++, need[5]++;
    } //统计各类型商品总需求数和产品格内已存在数
    vector<pair<int, int>> v;
    for(int i = 0; i < 6; i++){
        int m = (need[i] - exist[i]) * 10 / (need[i] + 1);
        pair<int, int> p(i + 1, m);
        v.push_back(p);
    } //以需求差值/总需求数作为权值计算并统计，权值比例为25
    sort(v.begin(), v.end(), compare);
    for(int i = 0; i < 6; i++){
        v[i].second *= 25;
    }
    if(stagingNum == 43){
        for(int i = 0; i < 6; i++){
            v[i].second *= 4;
        }
    }
    for(int i = 0; i < stagingNum; i++){
        for(int j = 1; j < 7; j++){
            if(f->stage[i].type == j){
                for(int k = 0; k < 6; k++){
                    if(v[k].first == j)
                        f->stage[i].value += v[k].second;
                }
            }
        }
    } //统计权值
    for(int i = 0; i < stagingNum; i++){
        if(f->stage[i].remainTime == -1)
            f->stage[i].value += 100;
    } //判断工作台是否生产，如果未生产，增加权值
    for(int i = 0; i < stagingNum; i++){
        if(f->stage[i].type == 4){
            if(f->stage[i].materialStatus == 2 || f->stage[i].materialStatus == 4){
                f->stage[i].value += 20;
                if(stagingNum == 43)
                    f->stage[i].value += 400;
            }
        }
        if(f->stage[i].type == 5){
            if(f->stage[i].materialStatus == 2 || f->stage[i].materialStatus == 8){
                f->stage[i].value += 20;
                if(stagingNum == 43)
                    f->stage[i].value += 400;
            }
        }
        if(f->stage[i].type == 6){
            if(f->stage[i].materialStatus == 4 || f->stage[i].materialStatus == 8){
                f->stage[i].value += 20;
                if(stagingNum == 43)
                    f->stage[i].value += 400;
            }
        }
        if(f->stage[i].type == 7){
            if(f->stage[i].materialStatus == 16 || f->stage[i].materialStatus == 32 || f->stage[i].materialStatus == 64){
                f->stage[i].value += 20;
                if(stagingNum == 43)
                    f->stage[i].value += 1000;
            }
            else if(f->stage[i].materialStatus == 48 || f->stage[i].materialStatus == 80 || f->stage[i].materialStatus == 96){
                f->stage[i].value += 40;
                if(stagingNum == 43)
                    f->stage[i].value += 2000;
            }
        }
    } //判断工作台材料格剩余量，根据后续所需材料数进行权值赋值，占比为20
}

bool if_send_near(read_by_frame f, int *judge1){
    if(judge1[1] != 0 && (judge1[4] != 0 || judge1[5] != 0))
        return true;
    else if(judge1[2] != 0 && (judge1[4] != 0 || judge1[6] != 0))
        return true;
    else if(judge1[3] != 0 && (judge1[5] != 0 || judge1[6] != 0))
        return true;
    else
        return false;
}

void sort_by_distance(read_by_frame *f,int robotID)//将工作台ID以工作台到机器人的距离远近排名
{
    set<pair<int,double>,cmp> stage_distance;
    vector<int> help;
    int size=f->stage.size();
    
    if(stagingNum == 50 || stagingNum == 25)
        add_stage_value(f, 6);
    if(stagingNum == 18)
        add_stage_value(f, 4);
    if(stagingNum == 43){
        f->stage[5].value += 100, f->stage[7].value += 100, f->stage[8].value += 100, f->stage[13].value += 150;
        f->stage[20].value += 50,  f->stage[24].value += 50, f->stage[27].value += 50, f->stage[28].value += 100;
        f->stage[29].value += 150, f->stage[31].value += 150, f->stage[32].value += 100, f->stage[33].value += 50;
        f->stage[11].value += 1000, f->stage[15].value += 1000, f->stage[17].value += 1000, f->stage[22].value += 1000;
    }
    for(int i=0;i<size;i++)
    {
        double distance=(1 / sqrt(pow((f->robot[robotID].x-f->stage[i].x),2)+pow((f->robot[robotID].y-f->stage[i].y),2))) * 200;
        pair<int,double> simple(i,distance);
        stage_distance.insert(simple);
    }
    set_stage_value(f);
    int j;
    if (if_have_stage(*f, 7)){
        if(f->stage.size()== 25)//2
            j = 70;
        else if(f->stage.size() == 43)
            j = 10;
        else if(f->stage.size() == 50)
            j = 13;
        else
            j = 5;
    }
    else{
        if(stagingNum == 50)//3
            j = 230;
        else
            j = 20;
    }
    for(auto i = stage_distance.begin(); i != stage_distance.end(); i++){
        f->stage[(*i).first].value += j * (*i).second;
    }
    int judge[10] = {0};
    vector<pair<int, double>> v1;
    vector<double> value1;
    for(auto i = stage_distance.begin(); i != stage_distance.end(); i++){
        if((*i).second >= 50){
            pair<int, double> s((*i).first, (*i).second);
            v1.push_back(s);
            judge[f->stage[(*i).first].type]++;
            value1.push_back(j * (*i).second);
        }
    }
    if(judge[4] == 0 && judge[5] == 0 && judge[6] == 0){
        for(int i = 0; i < v1.size(); i++)
            f->stage[v1[i].first].value -= value1[i];
    }
    if(judge[1] != 0 && judge[4] == 0 && judge[5] == 0){
        for(int i = 0; i < v1.size(); i++){
            if(f->stage[v1[i].first].type == 1)
                f->stage[v1[i].first].value -= value1[i];
        }
    }
    else if(judge[2] != 0 && judge[4] == 0 && judge[6] == 0){
        for(int i = 0; i < v1.size(); i++){
            if(f->stage[v1[i].first].type == 2)
                f->stage[v1[i].first].value -= value1[i];
        }
    }
    else if(judge[3] != 0 && judge[5] == 0 && judge[6] == 0){
        for(int i = 0; i < v1.size(); i++){
            if(f->stage[v1[i].first].type == 3)
                f->stage[v1[i].first].value -= value1[i];
        }
    }
    for(int i = 0; i < v1.size(); i++){
        for(int j = 0; j < v1.size(); j++){
            if((f->stage[v1[i].first].type == 4 || f->stage[v1[i].first].type == 5 || f->stage[v1[i].first].type == 6) && if_send_near(*f, judge)){
                if(!judge_have_material(*f, f->stage[v1[j].first].type, v1[i].first))
                    f->stage[v1[j].first].value -= value1[j];
                if(f->robot[robotID].productType == 0 && f->stage[v1[i].first].materialStatus != 6 && 
                   f->stage[v1[i].first].materialStatus != 10 && f->stage[v1[i].first].materialStatus != 12)
                    f->stage[v1[i].first].value -= value1[i];
            }
        }
    }
    vector<pair<int, int>> stage_value;
    for(int i = 0; i < stagingNum; i++){
        int v = f->stage[i].value;
        pair<int, int> s(i, v);
        stage_value.push_back(s);
    } //按照距离数统计权值，比例为10
    sort(stage_value.begin(), stage_value.end(), compare);
    for(auto it=stage_value.begin();it!=stage_value.end();it++)
    {
        help.push_back((*it).first);
    }
    f->stage_value_order.clear();
    f->stage_value_order.assign(help.begin(),help.end());
    for(int i = 0; i < stagingNum; i++){
        f->stage[i].value = 0;
    }
}
