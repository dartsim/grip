#include "GRIPApp.h"
#include "tabRipPlanner.h"

extern wxNotebook* tabView;

class RipTabPlanningApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new RipTabPlanning(tabView), wxT("RIP Planner"));
	}
};

IMPLEMENT_APP(RipTabPlanning)