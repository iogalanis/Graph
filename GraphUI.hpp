
#ifndef _GRAPH_UI_
#define _GRAPH_UI_



template <typename T>
int graphUI() {
  
  string option, line;
  int sum=0;
  bool digraph = false;
  
  cin >> option;
  if(!option.compare("digraph"))
    digraph = true;
  Graph<T> g(digraph);
  while(true) {
    
    std::stringstream stream;
    cin >> option;
    
    if(!option.compare("av")) {
      getline(std::cin, line);
      stream << line;
      T vtx(stream);
      if(g.addVtx(vtx))
        cout << "av " << vtx << " OK\n";
      else
        cout << "av " << vtx << " NOK\n";
      
    }
    else if(!option.compare("rv")) {
        getline(std::cin>>std::ws,line);
        stream<<line;
        T vtx(stream);
        
        if(g.rmvVtx(vtx))
            cout<<"rv "<<vtx<<" OK\n";
        else
            cout <<"rv "<<vtx<<" NOK\n";
        }
    else if(!option.compare("ae")) {
        getline(cin,line);
        
        line=line.substr(1,line.size()-1);
        
        stream<<line;
          
        T from(stream),to(stream);
         
        int distance;
        stream>>distance;
           
        if(g.addEdg(from,to,distance))
            cout<<"ae"<<" "<<from<<" "<<to<<" OK\n";
        else
            cout<<"ae"<<" "<<from<<" "<<to<<" NOK\n";
    }
    
    else if(!option.compare("re")) {

    }
    else if(!option.compare("dot")) {
      
    }
    else if(!option.compare("bfs")) {
      list<T> l;
      T next;
      cout << "\n----- BFS Traversal -----\n";
      getline(cin,line);
      
      stream<<line;
      T node(stream);
      
      l=g.bfs(node);
      
      while(!l.empty()){
          next=l.front();
          l.pop_front();
          if(l.empty())
              cout<<next;
          else
             cout<<next<<" -> ";
      }
      cout << "\n-------------------------\n";
    }
    else if(!option.compare("dfs")) {
     list<T> l;
      T next;
      
      cout << "\n----- DFS Traversal -----\n";
       getline(cin,line);
      
      stream<<line;
      T node(stream);
      
      l=g.dfs(node);
      
      while(!l.empty()){
          next=l.front();
          l.pop_front();
          if(l.empty())
              cout<<next;
          else
             cout<<next<<" -> ";
      }
      
      
      cout << "\n-------------------------\n";
    }
    else if(!option.compare("dijkstra")) {
      list<T> l;
      T next;
      getline(std::cin, line);
      stream << line;
      T from(stream);
      T to(stream);
      
      l=g.dijkstra(from,to);

      cout << "Dijkstra (" << from << " - " << to <<"): ";
      
      if(l.empty())
          cout<<endl;
      
      while(!l.empty()){
          next=l.front();
          l.pop_front();
          if(l.empty())
              cout<<next<<endl;
          else
             cout<<next<<", ";
      }
      
    }
    else if(!option.compare("mst")) {
        list<Edge<T>> mstList;
        Node<T> *from,*to;
        
        mstList=g.mst();
        cout << "\n--- Min Spanning Tree ---\n";
    
        if(mstList.empty())
            cout<<endl;
     
    
        while(!mstList.empty()){
            Edge<T> next=mstList.front();
            mstList.pop_front();
            from=g.findNode(next.from);
            to=g.findNode(next.to);
            
            if(from->pos<to->pos)
                cout<<next.from<<" -- "<<next.to<<" ("<<next.dist<<")"<<endl;
            else
                cout<<next.to<<" -- "<<next.from<<" ("<<next.dist<<")"<<endl;
            sum=sum + next.dist;
        }
      
        cout << "MST Cost: " << sum << endl;
    }
    else if(!option.compare("q")) {
      g.deleteEverything();
      cerr << "bye bye...\n";
      return 0;
    }
    else if(!option.compare("#")) {
      string line;
      getline(cin,line);
      cerr << "Skipping line: " << line << endl;
    }
    else {
      cout << "INPUT ERROR\n";
      return -1;
    }
  }
  return -1;  
}

#endif
