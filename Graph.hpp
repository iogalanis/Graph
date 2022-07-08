#ifndef _GRAPH_HPP_ 
#define _GRAPH_HPP_
#include <list>
#include <vector>
#include <iostream>
#include <climits>

using namespace std;

template<typename T>
struct Edge {
  T from;
  T to;
  int dist;
  Edge(T f, T t, int d): from(f), to(t), dist(d){}
  bool operator<(const Edge<T>& e) const{
    if(this->dist<e.dist)
        return true;
    else 
        return false;
  }
      
  bool operator>(const Edge<T>& e) const{
      if(this->dist<e.dist)
          return false;
      else 
          return true;
  }

  template<typename U>
  friend std::ostream& operator<<(std::ostream& out, const Edge<U>& e);
};

template <typename T>
struct Node{
    T data;
    int pos,dist,rnk=0;
    Node *next;
    Node *parent;
};
        
template<typename T>
std::ostream& operator<<(std::ostream& out, const Edge<T>& e) {
  out << e.from << " -- " << e.to << " (" << e.dist << ")";
  return out;
}

template <typename T>
class Graph {
   int N;
   T *init;
public:
    bool Directed;
  std::vector<Node<T>*> adjList;
  std::vector <Edge<T>> edges;
  std::vector<pair<T,int>> paired;
  std::list<T> shortList;
 
  void Union(Node <T> *x,Node<T> *y);
  Node <T> *findRoot(Node <T> *node);
  Node <T> *findNode(T info);
  void  shortestPath(T parent[],T info,const T from);
  void resetDist();
  bool isAdj(const T&  t1, const T& t2);
  Node <T>* minDistance(bool *sptSet);
  void deleteEverything();
  void doDfs(const T& info,bool *visited,list<T> *dfsList)const;
  void DeleteNode(Node<T> *curr);
  void InsertNode(const T& from,const T& to);
  int returnInt(vector<pair<T,int>> vect,const T &info) const;
  Graph(bool isDirected );
  ~Graph();
  bool addVtx(const T& info);
  bool contains(const T& info);
  bool rmvVtx(const T& info);
  bool addEdg(const T& from, const T& to, int distance);
  bool rmvEdg(const T& from, const T& to);
  std::list<T> dfs(const T& info) const;
  std::list<T> bfs(const T& info) const;
  std::list<Edge<T>> mst();
  
  bool print2DotFile(const char *filename) const;
  std::list<T> dijkstra(const T& from, const T& to);

};
template <typename T>
Graph<T>::~Graph(){
    
}
template <typename T>
Graph<T>::Graph(bool isDirected){
    Directed=isDirected;
    N=0;
}

template <typename T>
bool Graph<T>::contains(const T& info){
    if(N==0){
        return false;
    }
    for(auto i=adjList.begin();i!=adjList.end();++i){
        if((*i)->data==info){
            return true;
        }
    }
    return false;
}

template <typename T>
bool Graph<T>::addVtx(const T& info){
    pair<T,int> newPair;
    if(contains(info)){
        return false;
    }
    
    Node<T> *newNode=new Node<T>;
    newNode->data=info;
    newNode->next=nullptr;
    newNode->pos=N++;
    newNode->dist=INT_MAX;
    newNode->parent=newNode;
    newNode->rnk=0;
    
    adjList.push_back(newNode);
    
     newPair=make_pair(info,adjList.size()-1);
     paired.push_back(newPair);
    
    return true;
}

template <typename T>
void Graph<T>::DeleteNode(Node<T> *curr){
    if(curr->next!=nullptr){
        DeleteNode(curr->next);
        delete curr;
        return;
    }
    else{
        delete curr;
        return;
    }
}
    

template <typename T>
bool Graph<T>::rmvVtx(const T& info){
    Node<T> *deleteNode,*curr,*prev;

    
   for(auto i=adjList.begin();i!=adjList.end();++i){
       if((*i)->data==info){
            deleteNode=(*i);
            adjList.erase(i);
            DeleteNode(deleteNode);
            break;
       }
   }
   for(auto i=edges.begin();i!=edges.end();++i){
       if(i->from==info){
           edges.erase(i);
       }
       else if(i->to==info){
            for(auto j=adjList.begin();j!=adjList.end();++j){
                if((*j)->data==i->from){
                    curr=(*j);
                    break;
                }
            }
            while(curr->data!=info){
                prev=curr;
                curr=curr->next;
            }
            if(curr->next==nullptr){
                prev->next=nullptr;
                delete curr;
            }
            else{
                prev->next=curr->next;
                delete curr;
            }
       }
   }
            
           
   return true;
}

template <typename T>
void Graph<T>::InsertNode(const T& from,const T &to){
    Node<T> *nxt,*toNode=new Node<T>,*prev;
       
    
    for(auto j=adjList.begin();j!=adjList.end();++j){
        if((*j)->data==to){
            toNode->data=(*j)->data;
            toNode->next=nullptr;
            toNode->pos=(*j)->pos;
            toNode->dist=INT_MAX;
        }
    }
    
    for(auto j = adjList.begin();j!=adjList.end();++j){
        if((*j)->data == from){
            if((*j)->next==nullptr){
                (*j)->next=toNode;
                break;
            }
            else{
                prev=(*j);
                nxt=(*j)->next;
                while(1){
                    if(toNode->pos<nxt->pos){
                        prev->next=toNode;
                        toNode->next=nxt;
                        break;
                        }
                    else if(toNode->pos> nxt->pos && nxt->next==nullptr){
                         nxt->next=toNode;
                         break;
                     }
                   
                     else{
                         prev=nxt;
                         nxt=nxt->next;
                     }
                    }
                }
            }
    }
}
    

template <typename T>
bool Graph<T>::addEdg(const T& from, const T& to, int distance){
    
    if(!contains(from)){
        return false;
    }
    if(!contains(to)){
        return false;
    }
    
     for(auto i=edges.begin();i!=edges.end();++i){
        if(from== i->from && to==i->to){
            return false;
        }
        else if(from==i->to && to==i->from && !Directed)
            return false;
    }
    
    Edge<T> newEdge(from,to,distance);
    edges.push_back(newEdge);
    
    InsertNode(from,to);
    
    if(!Directed){
        InsertNode(to,from);
    }
    
    
    
    return true ;
    
}
template <typename T>
bool Graph<T>::rmvEdg(const T& from, const T& to){
    Edge<T> *deleteEdge;
    
    for(auto i=edges.begin();i!=edges.end();++i){
        if(from== i->from && to==i->to){
            
            edges.erase(i);
            
        }
    }
    for(auto j=adjList.begin(); j!=adjList.end();++j){
        if(j->data==from){
            for(auto z=j->connectedEdges.begin();z!=j->connectedEdges.end();++j){
                if(z->to==to){
                    deleteEdge=&(*z);
                    j->connectedEdges.erase(z);
                    delete deleteEdge;
                    return true;
                }
            }
        }
    }
    return false;
    
}
template <typename T>
int Graph<T>::returnInt(vector<pair<T,int>> vect,const T& info)const{
    for(auto i=vect.begin();i!=vect.end();++i){
        if(i->first==info){
            return i->second;
        }
    }
    return -1;
}
    
template <typename T>
std::list<T> Graph<T>::bfs(const T& info) const{
  
    T nxt;
    int i=0,next;
    bool  *visited=new bool[N];

    list<T> l,q;
    Node<T> *ptr,*curr;
    
    for( i=0;i<N;i++){
        visited[i]=false;
    }
    list<T> queue;
    
    next=returnInt(paired,info);
    visited[next]=true;
    queue.push_back(info);
    
    
    while(!queue.empty()){
        nxt=queue.front();
        l.push_back(nxt);
        queue.pop_front();
        
        
        for(auto j=adjList.begin();j!=adjList.end();++j){
            if((*j)->data==nxt){
                curr=(*j);
            }
        }
        
        if(curr->next!=nullptr){
             ptr=curr->next;
          
            while(1){
                next=returnInt(paired,ptr->data);
                if(!visited[next]){
                    visited[next]=true;
                    queue.push_back(ptr->data);
                }
                if(ptr->next==nullptr)
                    break;
                else
                    ptr=ptr->next;
                
            }
        }
    }
    delete [] visited;
    return l;
}

template <typename T>
void Graph<T>::doDfs(const T& info,bool *visited ,list<T> *dfsList)const {
    T nxt;
    int next;
    Node<T> *curr;
    
    next=returnInt(paired,info);
    
    visited[next]=true;
    dfsList->push_back(info);
    
    for(auto i=adjList.begin();i!=adjList.end();++i){
        if((*i)->data==info){
            curr=(*i);
        }
    }
    
    while(true){
        if(curr->next==nullptr){
            break;
            
        }
        else {
            curr=curr->next;
            next=returnInt(paired,curr->data);
            if(!visited[next])
                doDfs(curr->data,visited,dfsList);
        }
    }
}

template <typename T>
std::list<T> Graph<T>::dfs(const T& info) const{
   
    list<T> dfsList;
    
    bool visited[N];
    for(auto i=0; i<N ;i++){
        visited[i]=false;
    }
    
    doDfs(info,visited,&dfsList);
    
    return dfsList;
    
}
template <typename T>
void Graph<T>::deleteEverything(){
    for(auto i=adjList.begin();i!=adjList.end();++i){
        DeleteNode(*i);
    }
}
template <typename T>
Node <T> *Graph<T>::minDistance(bool *sptSet){
    int min=INT_MAX,next;
    Node <T> *curr;
    
    for(auto i=adjList.begin();i!=adjList.end();++i){
        next=returnInt(paired,(*i)->data);
        if(sptSet[next]==false && (*i)->dist<=min){
            min=(*i)->dist;
            curr=*i;
        }
    }
    
    return curr;
}
template<typename T> 
bool Graph<T>::isAdj(const T& t1, const T& t2){
    Node <T> *from,*curr;
    if(t1==t2)
        return true;
    for(auto i=adjList.begin();i!=adjList.end();++i){
        if(t1==(*i)->data){
            from=(*i);
        }
    }
    
    if(from->next!=nullptr){
        curr=from->next;
        while(true){
            if(curr->data==t2)
                return true;
            else if(curr->next==nullptr)
                break;
            else 
                curr=curr->next;
        }
    }
    return false ;
}

template <typename T>
void Graph<T>::resetDist(){
    Node<T> *curr;
    for(auto i=adjList.begin();i!=adjList.end();++i){
        curr=*i;
        curr->dist=INT_MAX;
        
        if(curr->next!=nullptr){
            curr=curr->next;
            while(true){
                curr->dist=INT_MAX;
                
                if(curr->next==nullptr)
                    break;
                else 
                    curr=curr->next;
            }
        }
    }
    
}
template <typename T>                
void Graph<T>::shortestPath(T parent[], T info,const T from ){
    int next;
    next=returnInt(paired,info);
    
    if(parent[next]==from || parent[next]==*init )
        return ;
    
    shortestPath(parent,parent[next],from);
    shortList.push_back(parent[next]);
}
    

template <typename T>
std::list<T> Graph<T>::dijkstra(const T& from, const T& to){
    bool *sptSet=new bool[N];
    Node <T> *f,*min,*curr;
    int next,dist;
    T *parent=new T[N];
    list<T> l;
    shortList.clear();
    std::stringstream stream;
    stream<<-1;
     init=new T(stream);

    
    for (int i=0; i<N; i++){
       sptSet[i]=false;
       parent[i]=*init;
    }
    
    for(auto i=adjList.begin(); i!=adjList.end();++i){
        if((*i)->data == from)
            f=*i;
    }
    
    f->dist=0;
    shortList.push_back(from);
    
    int k;
    for( k=0; k<N; k++){
        min=minDistance(sptSet);
        next=returnInt(paired,min->data);
        
        sptSet[next]=true;
        
        if(min->data==to)
            break;
        
        if(min->next!=nullptr){
            curr= min->next;
            while(true){
                next=returnInt(paired,curr->data);
                for(auto i=edges.begin();i!=edges.end();++i){
                    if(i->from== min->data && i->to ==curr->data){
                        dist=i->dist;
                    }
                }
                if(!sptSet[next] && min->dist!=INT_MAX && min->dist + dist <curr->dist){
                    curr->dist=min->dist +dist;
                    parent[next]=min->data;
                    
                    for(auto k=adjList.begin();k!=adjList.end();++k){
                        if((*k)->data==curr->data){
                            (*k)->dist=curr->dist;
                            break;
                        }
                    }
                }
                
                if(curr->next==nullptr)
                    break;
                else 
                    curr=curr->next;
            
            }
        }
    }
    shortestPath(parent,min->data,from);

    if(shortList.size()==1){
        shortList.clear();
    }
    
    resetDist();
    delete [] parent;
    delete init;
    return shortList;
}

template <typename T> 
Node <T>* Graph<T>::findNode(T info){
    Node <T> *curr;
    for(auto i=adjList.begin();i!=adjList.end();++i){
        if(info==(*i)->data)
            curr=*i;
    }
    return curr;
}
    
template<typename T> 
Node <T> *Graph<T>::findRoot(Node <T> *node){
    Node<T> *curr;
    curr=node;
    while(curr->parent !=curr){
        curr=curr->parent;
    }
    
    return curr;
}
template<typename T>
void Graph<T>::Union(Node <T> *x,Node<T> *y){
    Node<T> *xroot,*yroot;
    
    xroot=findRoot(x);
    yroot=findRoot(y);
    
    if(xroot->rnk< yroot->rnk)
        xroot->parent=yroot;
    else if(xroot->rnk>yroot->rnk)
        yroot->parent=xroot;
    else{
        yroot->parent=xroot;
        xroot->rnk++;
    }
}
    

template <typename T>
std::list<Edge<T>> Graph<T>::mst(){
   list<Edge<T>> edgesList,mstList;
   Node <T> *from,*to,*x,*y;
   Edge<T> *edge;
   
  for(auto i=edges.begin();i!=edges.end();++i)
      edgesList.push_back(*i);
  
  edgesList.sort();
  
  for(auto i=edgesList.begin();i!=edgesList.end();++i){
      edge =&*i;
      
      from=findNode(edge->from);
      to=findNode(edge->to);
      
      x=findRoot(from);
      y=findRoot(to);
      
      if(x!=y){
          mstList.push_back(*edge);
          Union(x,y);
      }
  }
          
      
  return mstList;    
      
}

    
    
    
    
    

#endif
