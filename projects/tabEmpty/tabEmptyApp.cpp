#include "GRIPApp.h"
#include "tabEmpty.h"
/* To include Planning Tab, ensure that Tab/PlanningTab is present in SRC_WX for
 * libGRIP/CMakeLists.txt. Then uncomment the two lines below */

// #include <Tabs/PlanningTab.h>

extern wxNotebook* tabView;

class EmptyTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new EmptyTab(tabView), wxT("EmptyTab"));
		// tabView->AddPage(new PlanningTab(tabView), wxT("PlanningTab"));
	}
};

IMPLEMENT_APP(EmptyTabApp)

