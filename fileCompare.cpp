// reading a text file
#include <iostream>
#include <fstream>
#include <string>
#include <vector>


using namespace std;

struct InventoryItem{
    int count;
    string item;
};

void fileCompare(void);
int main (void)
{
   fileCompare();
   return 0;
}

void fileCompare(void)
{
    vector<InventoryItem> oldItems;

    string line;

  ifstream myfile ("inventory.txt");
  if (myfile.is_open())
  {
    for (int lineno = 0; getline (myfile,line) && lineno < 7; lineno++){
          char newLine [100];
          int newValue;

          if(line == "Green Shelf:")
          {
            break;
          }

          if (lineno >= 3 ){

              int i = 3;

              while( line[i] != 32 || line[i+1] != 45)
              {
                newLine[i-3] = line[i];
                i++;
              }

              newValue = line[i+3] - 48;

              InventoryItem newItem;
              newItem.count = newValue;
              newItem.item = newLine;
              oldItems.push_back(newItem);
          }
    }

    vector<InventoryItem> comparisonVector;
    int placeHolder;

    // DELETE LATER
    vector<InventoryItem> all_shelves = oldItems;
    all_shelves[2].count = 7;

    InventoryItem adder;
    adder.count = 3;
    adder.item = "dots";
    oldItems.push_back(adder);

    InventoryItem minuser;
    minuser.count = 3;
    minuser.item = "Apple Peels";
    all_shelves.push_back(minuser);

    int foundItem = 0;

    for(int old = 0; old < oldItems.size() ; old++)
    {

        int valueCompare = oldItems[old].count;
        string name = oldItems[old].item;

        for(int newer = 0; newer < all_shelves.size() ; newer++)
        {
            InventoryItem newItem;

            if(name == all_shelves[newer].item)
            {
              newItem.item = name;
              newItem.count = all_shelves[newer].count - valueCompare;
              foundItem = 1;
              comparisonVector.push_back(newItem);
            }



        }

        if (foundItem == 0)
        {
              InventoryItem newItem;
              newItem.item = name;
              newItem.count = -1 * valueCompare;
              comparisonVector.push_back(newItem);
        }
        foundItem = 0;
    }


    for(int newer = 0; newer < all_shelves.size() ; newer++)
    {
        int valueCompare = all_shelves[newer].count;
        string name = all_shelves[newer].item;

        for(int old = 0; old < oldItems.size() ; old++)
        {
            if(name == oldItems[newer].item)
            {
              foundItem = 1;
            }
        }

        if (foundItem == 0)
        {
              InventoryItem newerItem;
              newerItem.item = name;
              newerItem.count = valueCompare;
              comparisonVector.push_back(newerItem);
        }
        foundItem = 0;
    }


    std::string home = getenv("HOME");
    std::string changes_inventory_file = home + "/changes.txt";
    ofstream f_out(changes_inventory_file.c_str());

    f_out << "Changes to Inventory since Last Scan:" << endl;

  for(int i = 0; i<comparisonVector.size(); i++)
  {
        f_out << comparisonVector[i].item << ": ";
        if (comparisonVector[i].count > 0){
            f_out << "+" << comparisonVector[i].count <<endl;
        }
        else {
            f_out << comparisonVector[i].count<<endl;
        }
  }

}

}
