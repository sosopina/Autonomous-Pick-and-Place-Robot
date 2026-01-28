import numpy as np
def put_in_range(list_alpha):
    list_alpha=np.array(list_alpha)
    k=np.argwhere(np.array(list_alpha)>=np.pi)
    list_alpha[k]=list_alpha[k]-(2*np.pi);
    k=np.argwhere(list_alpha<-np.pi)
    list_alpha[k]=list_alpha[k]+(2*np.pi);
    return list_alpha 

def get_sol_type2(X,Y,Z):
    #return all the solutions alpha[i=0..nbSols] of the equation X.sin(alpha)+Y.cos(alpha)==Z
    normXY = np.sqrt(X*X+Y*Y)
    if normXY<1e-10:
        alpha=[] # any solution should be  ok if Z==0 
        return alpha
    if abs(Z)>normXY*(1+1e-6):
        alpha=[]
        return alpha
    Zn=max([Z/normXY,-1])
    Zn=min([Zn,1])
    beta=np.arctan2(Y,X)    
    if Zn>=1:
       alpha_beta=np.pi/2
       alpha=alpha_beta-beta
       return put_in_range([alpha])
    if Zn<=-1:
       alpha_beta=-np.pi/2
       alpha=alpha_beta-beta
       return put_in_range([alpha])
    alpha_beta1=np.arcsin(Zn)
    alpha_beta2=np.pi-alpha_beta1
    alpha1=  alpha_beta1-beta
    alpha2=  alpha_beta2-beta
    return put_in_range([alpha1,alpha2])  


def get_rot_x(alpha):
    c=np.cos(alpha)
    s=np.sin(alpha)
    iTj=np.array([
        [1, 0, 0,0],
        [0, c,-s,0],
        [0, s, c,0],
        [0, 0, 0,1]])
    return iTj    
def get_rot_y(alpha):
    c=np.cos(alpha)
    s=np.sin(alpha)
    iTj=np.array([
        [c , 0, s,0],
        [0 , 1, 0,0],
        [-s, 0, c,0],
        [0 , 0, 0,1]])
    return iTj    
def get_rot_z(alpha):
    c=np.cos(alpha)
    s=np.sin(alpha)
    iTj=np.array([
        [c ,-s, 0,0],
        [s , c, 0,0],
        [0 , 0, 1,0],
        [0 , 0, 0,1]])
    return iTj    
def get_trans(x,y,z):
    iTj=np.array([
        [1 , 0, 0, x],
        [0,  1, 0, y],
        [0 , 0, 1, z],
        [0 , 0, 0, 1]
        ])
    return iTj    
def get_R(iTj):
    iRj=iTj[0:3,0:3]
    return iRj    
def get_O(iTj):
    iOj=iTj[0:3,3:4]
    return iOj    
def get_invT(iTj):
    iRj=get_R(iTj)
    iOj=get_O(iTj)
    jRi=np.transpose(iRj)
    jOi=-multMatrix(jRi,iOj)
    jTi=np.concatenate((jRi,jOi),axis=1) # concatenation colonne
    lastRow=np.array([[0,0,0,1]]) 
    jTi=np.concatenate((jTi,lastRow),axis=0) # concatenation ligne
    return jTi

def multMatrix(iTj,jTk):
    iTk=np.matmul(iTj,jTk)   
    return iTk
def get_Pi(iTj,Pj):
    lastRow=np.array([[1]]) 
    Pj1=np.concatenate((Pj,lastRow),axis=0) # concatenation ligne
    Pi=multMatrix(iTj[0:3,:],Pj1)
    return Pi
def get_Vi(iTj,Vj):
    lastRow=np.array([[0]]) 
    Vj0=np.concatenate((Vj,lastRow),axis=0) # concatenation ligne
    Vi=multMatrix(iTj[0:3,:],Vj0)
    return Vi

def get_vec3(x,y,z):
    vec3=np.array([[x],[y],[z]]) 
    return vec3
