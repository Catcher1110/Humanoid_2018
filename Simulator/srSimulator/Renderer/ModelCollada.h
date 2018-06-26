#ifndef ModelCollada_H
#define ModelCollada_H

#include <stdio.h>
#include <vector>

// I decided to use my GLTexture class b/c adding all of its functions
// Would have greatly bloated the model class's code
// Just replace this with your favorite texture class
#include "srg/srgL.h"
#include "Renderer/Texture.h"

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>


class ModelCollada  
{
private:


void set_float4(float f[4], float a, float b, float c, float d);
void color4_to_float4(const aiColor4D *c, float f[4]);
void apply_material(const struct aiMaterial *mtl);
void recursive_render (const struct aiScene *sc, const struct aiNode* nd);
    const struct aiScene* scene;

    double ratio_;
    GLuint scene_list;
    //Added not by Matthew. This is so we can modify the path length easily.
    static const int PATH_LENGTH = 2000;

    struct Vector {
        float x;
        float y;
        float z;
    };

    // Vertex struct to make code easier to read in places
    struct Vertex {
        float x;
        float y;
        float z;
    };

    struct Triangle {
        double v1[3];
        double v2[3];
        double v3[3];
    };

    // Color struct holds the diffuse color of the material
    struct Color4i {
        unsigned char r;
        unsigned char g;
        unsigned char b;
        unsigned char a;
    };

    // Holds the material info
    // TODO: add color support for non textured polys
    struct Material {
        char	name[PATH_LENGTH];	// The material's name
        Texture	tex;				// The texture (this is the only outside reference in this class)
        bool	textured;			// whether or not it is textured
        Color4i	color;
    };

    // Every chunk in the 3ds file starts with this struct
    struct ChunkHeader {
        unsigned short id;	// The chunk's id
        unsigned int  len;	// The length of the chunk
    };
    struct ChunkHeader_short {
        unsigned short id;	// The chunk's id
        unsigned short  len;	// The length of the chunk
    };

    // I sort the mesh by material so that I won't have to switch textures a great deal
    struct MaterialFaces {
        unsigned short *subFaces;	// Index to our vertex array of all the faces that use this material
        int numSubFaces;			// The number of faces
        int MatIndex;				// An index to our materials
    };

    // The 3ds file can be made up of several objects
    struct Object {
        char	name[PATH_LENGTH];	// The object name
        float	*Vertexes;			// The array of vertexes
        float	*Normals;			// The array of the normals for the vertexes
        float	*TexCoords;			// The array of texture coordinates for the vertexes
        unsigned short *Faces;		// The array of face indices
        int		numFaces;			// The number of faces
        int		numMatFaces;		// The number of differnet material faces
        int		numVerts;			// The number of vertexes
        int		numTexCoords;		// The number of vertexes
        bool	textured;			// True: the object has textures
        MaterialFaces *MatFaces;	// The faces are divided by materials
        Vector	pos;				// The position to move the object to
        Vector	rot;				// The angles to rotate the object
    };

public:
    // Transformation
    float _T[16];

    GLuint	modelDL;
    GLuint	colDL;
    char	path[PATH_LENGTH];
    char	modelname[PATH_LENGTH];
    int		numObjects;			// Total number of objects in the model
    int		numMaterials;		// Total number of materials in the model
    int		totalVerts;			// Total number of vertexes in the model
    int		totalFaces;			// Total number of faces in the model
    bool	shownormals;		// True: show the normals
    Material *Materials;		// The array of materials
    Object	*Objects;			// The array of objects in the model
    Vector	pos;				// The position to move the model to
    Vector	rot;				// The angles to rotate the model
    float	scale;				// The size you want the model scaled to
    bool	lit;				// True: the model is lit
    bool	visible;			// True: the model gets rendered
    double	Stri[3][3];			// Storing the triangle
    FILE	*bin3ds;			// The binary 3ds file
    Vector	max, min;			// max/min vertex.

public:
    ModelCollada();		// Constructor
    virtual ~ModelCollada();		// Destructor

    void	ReportTriangles(vector<Triangle> *trigs);
    void    Load(char *name);	// Loads a model
    void	Draw();				// Draws the model


private:
    void IntColorChunkProcessor(long length, long findex, int matindex);
    void FloatColorChunkProcessor(long length, long findex, int matindex);
    // Processes the Main Chunk that all the other chunks exist is
    void MainChunkProcessor(long length, long findex);
    // Processes the model's info
    void EditChunkProcessor(long length, long findex);
			
    // Processes the model's materials
    void MaterialChunkProcessor(long length, long findex, int matindex);
    // Processes the names of the materials
    void MaterialNameChunkProcessor(long length, long findex, int matindex);
    // Processes the material's diffuse color
    void DiffuseColorChunkProcessor(long length, long findex, int matindex);
    // Processes the material's texture maps
    void TextureMapChunkProcessor(long length, long findex, int matindex);
    // Processes the names of the textures and load the textures
    void MapNameChunkProcessor(long length, long findex, int matindex);
			
    // Processes the model's geometry
    void ObjectChunkProcessor(long length, long findex, int objindex);
    // Processes the triangles of the model
    void TriangularMeshChunkProcessor(long length, long findex, int objindex);
    // Processes the vertexes of the model and loads them
    void VertexListChunkProcessor(long length, long findex, int objindex);
    // Processes the texture cordiantes of the vertexes and loads them
    void TexCoordsChunkProcessor(long length, long findex, int objindex);
    // Processes the faces of the model and loads the faces
    void FacesDescriptionChunkProcessor(long length, long findex, int objindex);
    // Processes the materials of the faces and splits them up by material
    void FacesMaterialsListChunkProcessor(long length, long findex, int objindex, int subfacesindex);

    // Calculates the normals of the vertexes by averaging
    // the normals of the faces that use that vertex
    void CalculateNormals();
};

#endif
