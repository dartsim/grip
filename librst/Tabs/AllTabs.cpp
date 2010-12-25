#include <Tabs/AllTabs.h>
#include <Tabs/InspectorTab.h>

void addAllTabs() {
	ADD_TAB(InspectorTab,wxT("Inspector"))
	tabView->SetSelection(0);
}
