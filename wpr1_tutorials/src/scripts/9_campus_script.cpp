#include "9_campus_script.h"
CCampusScript::CCampusScript()
{
    
}

CCampusScript::~CCampusScript()
{

}

void CCampusScript::Queue()
{
    stAct newAct;

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "智慧校园 项目开始";
    newAct.nDuration = 3;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "机器人";
    arAct.push_back(newAct);
    
    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "你好,小明,今天要借哪本书?";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "王者荣耀";
    newAct.nDuration = 3;
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "好的,这就给你去取";
    newAct.nDuration = 4;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "bookshelf";
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "reception";
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "你好,书已经给你取来.再见!";
    newAct.nDuration = 4;
    arAct.push_back(newAct);
}