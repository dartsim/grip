#include "RSTApp.h"
#include "EmptyTab.h"
/* To include Planning Tab, ensure that Tab/PlanningTab is present in SRC_WX for
 * librst/CMakeLists.txt. Then uncomment the two lines below */

// #include <Tabs/PlanningTab.h>

extern wxNotebook* tabView;

class EmptyTabApp : public RSTApp {
	virtual void AddTabs() {
		tabView->AddPage(new EmptyTab(tabView), wxT("EmptyTab"));
		// tabView->AddPage(new PlanningTab(tabView), wxT("PlanningTab"));
	}
};

IMPLEMENT_APP(EmptyTabApp)

