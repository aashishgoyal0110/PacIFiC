#include "BoxShell.hh"
#include "GrainsExec.hh"
#include "PointC.hh"
#include "ContactBuilderFactory.hh"
#include "ObstacleBuilderFactory.hh"
#include "Box.hh"
#include "Rectangle.hh"


// ----------------------------------------------------------------------------
// Constructor with an XML node as an input parameter
BoxShell::BoxShell( DOMNode* root ) :
  CompositeObstacle( "shell" )
{    
  assert( root != NULL );
  
  // Type
  m_type = "BoxShell";

  // Name
  m_name = ReaderXML::getNodeAttr_String( root, "name" );
  
  // Geometry
  DOMNode* nGeom = ReaderXML::getNode( root, "Geometry" );
  m_lx = ReaderXML::getNodeAttr_Double( nGeom, "LX" );
  m_ly = ReaderXML::getNodeAttr_Double( nGeom, "LY" );
  m_lz = ReaderXML::getNodeAttr_Double( nGeom, "LZ" );
  m_shellWidth = ReaderXML::getNodeAttr_Double( nGeom, "Width" );
  double crust_thickness = ReaderXML::getNodeAttr_Double( nGeom, 
  	"CrustThickness" );
  m_box = false;
  if ( ReaderXML::hasNodeAttr( nGeom, "ElementaryObstacle" ) )
  {
    string selem = ReaderXML::getNodeAttr_String( nGeom, "ElementaryObstacle" );
    if ( selem == "Box" ) m_box = true;
  }   

  // Center of mass and angular position of the cylindrical shell
  m_geoRBWC->getTransform()->load( root );
  
  // Crust thickness 
  m_geoRBWC->setCrustThickness( crust_thickness );  

  // Material
  DOMNode* materiau_ = ReaderXML::getNode( root, "Material" );
  m_materialName = ReaderXML::getNodeValue_String( materiau_ );
  ContactBuilderFactory::defineMaterial( m_materialName, true );

  // Obstacle to transfer to the fluid
  bool transferToFluid = false;
  DOMNode* status = ReaderXML::getNode( root, "Status" );
  if ( status )
    transferToFluid = ReaderXML::getNodeAttr_Int( status, "ToFluid" );

  // Create elementary box obstacles 
  Vector3 zerotranslation;    
  if ( m_box )
  {
    // Behind
    Box* box = new Box( m_lx + 2. * m_shellWidth, m_ly + 2. * m_shellWidth, 
    	m_shellWidth );
    RigidBodyWithCrust* geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    Obstacle* sbox = new SimpleObstacle( m_name + "_behind", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    Point3 cg( 0., 0., - 0.5 * m_lz - 0.5 * m_shellWidth );
    sbox->setPosition( cg );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox );
    
    // Front
    box = new Box( m_lx + 2. * m_shellWidth, m_ly + 2. * m_shellWidth, 
    	m_shellWidth );
    geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    sbox = new SimpleObstacle( m_name + "_front", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    cg.setValue( 0., 0., 0.5 * m_lz + 0.5 * m_shellWidth );
    sbox->setPosition( cg );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox );
    
    // Bottom
    box = new Box( m_lx + 2. * m_shellWidth, m_shellWidth, 
    	m_lz + 2. * m_shellWidth );
    geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    sbox = new SimpleObstacle( m_name + "_bottom", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    cg.setValue( 0., - 0.5 * m_ly - 0.5 * m_shellWidth, 0. );
    sbox->setPosition( cg );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox );
    
    // Top
    box = new Box( m_lx + 2. * m_shellWidth, m_shellWidth, 
    	m_lz + 2. * m_shellWidth );
    geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    sbox = new SimpleObstacle( m_name + "_top", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    cg.setValue( 0., 0.5 * m_ly + 0.5 * m_shellWidth, 0. );
    sbox->setPosition( cg );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox );
    
    // Left
    box = new Box( m_shellWidth, m_ly + 2. * m_shellWidth,  
    	m_lz + 2. * m_shellWidth );
    geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    sbox = new SimpleObstacle( m_name + "_left", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    cg.setValue( - 0.5 * m_lx - 0.5 * m_shellWidth, 0., 0. );
    sbox->setPosition( cg );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox );
    
    // Right
    box = new Box( m_shellWidth, m_ly + 2. * m_shellWidth,  
    	m_lz + 2. * m_shellWidth );
    geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    sbox = new SimpleObstacle( m_name + "_right", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    cg.setValue( 0.5 * m_lx + 0.5 * m_shellWidth, 0., 0. );
    sbox->setPosition( cg );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox );                     
  }
  else
  {
    Matrix mrot;

    // Behind
    Rectangle* box = new Rectangle( m_lx + 2. * m_shellWidth, 
    	m_ly + 2. * m_shellWidth, m_shellWidth, RVE_ZMINUS );
    RigidBodyWithCrust* geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    Obstacle* sbox = new SimpleObstacle( m_name + "_behind", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    Point3 cg( 0., 0., - 0.5 * m_lz );
    sbox->setPosition( cg );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox );
    
    // Front
    box = new Rectangle( m_lx + 2. * m_shellWidth, 
    	m_ly + 2. * m_shellWidth, m_shellWidth, RVE_ZPLUS );
    geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    sbox = new SimpleObstacle( m_name + "_front", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    cg.setValue( 0., 0., 0.5 * m_lz );
    sbox->setPosition( cg );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox );
    
    // Bottom
    box = new Rectangle( m_lx + 2. * m_shellWidth, 
    	m_lz + 2. * m_shellWidth, m_shellWidth, RVE_ZMINUS );
    geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    sbox = new SimpleObstacle( m_name + "_bottom", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    cg.setValue( 0., - 0.5 * m_ly, 0. );
    sbox->setPosition( cg );
    mrot.setValue( 1., 0., 0., 0., 0., 1., 0., -1., 0. );
    sbox->getRigidBody()->getTransform()->setBasis( mrot );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox ); 
    
    // Top
    box = new Rectangle( m_lx + 2. * m_shellWidth, 
    	m_lz + 2. * m_shellWidth, m_shellWidth, RVE_ZMINUS );
    geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    sbox = new SimpleObstacle( m_name + "_top", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    cg.setValue( 0., 0.5 * m_ly, 0. );
    sbox->setPosition( cg );
    mrot.setValue( 1., 0., 0., 0., 0., -1., 0., 1., 0. );
    sbox->getRigidBody()->getTransform()->setBasis( mrot );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox );
    
    // Left
    box = new Rectangle( m_lz + 2. * m_shellWidth, 
    	m_ly + 2. * m_shellWidth, m_shellWidth, RVE_ZMINUS );
    geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    sbox = new SimpleObstacle( m_name + "_left", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    cg.setValue( - 0.5 * m_lx, 0., 0. );
    sbox->setPosition( cg );
    mrot.setValue( 0., 0., 1., 0., 1., 0., -1., 0., 0. );
    sbox->getRigidBody()->getTransform()->setBasis( mrot );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox );
    
    // Right
    box = new Rectangle( m_lz + 2. * m_shellWidth, 
    	m_ly + 2. * m_shellWidth, m_shellWidth, RVE_ZMINUS );
    geoRBWC_box = new RigidBodyWithCrust( box, Transform(),
  	false, crust_thickness );
    sbox = new SimpleObstacle( m_name + "_right", geoRBWC_box, 
	m_materialName, transferToFluid, true );
    cg.setValue( 0.5 * m_lx, 0., 0. );
    sbox->setPosition( cg );
    mrot.setValue( 0., 0., -1., 0., 1., 0., 1., 0., 0. );    
    sbox->getRigidBody()->getTransform()->setBasis( mrot );
    sbox->getRigidBody()->composeLeftByTransform( *m_geoRBWC->getTransform() );
    sbox->Translate( zerotranslation );
    m_obstacles.push_back( sbox );                   
  }
  
  computeVolumeCenterOfMass();
}




// ----------------------------------------------------------------------------
// Constructor with name as input parameter
BoxShell::BoxShell( string const& s )
  : CompositeObstacle( s )
{
  m_type = "BoxShell";  
} 




// ----------------------------------------------------------------------------
// Destructor
BoxShell::~BoxShell()
{}




// ----------------------------------------------------------------------------
// Outputs the cylindrical shell for reload
void BoxShell::write( ostream& fileSave ) const
{
  fileSave << "<Composite> " << m_name << " " << m_type << endl;
  fileSave << "*Properties " << m_lx << " " << m_ly << " " 
  	<< m_lz << " " << m_shellWidth << endl;  
  if ( m_CompositeObstacle_id ) m_torsor.write( fileSave );
  list<Obstacle*>::const_iterator obstacle;
  for (obstacle=m_obstacles.begin(); obstacle!=m_obstacles.end(); obstacle++)
  {
    (*obstacle)->write( fileSave );
    fileSave << endl;
  }
  fileSave << "</Composite>";
}




// ----------------------------------------------------------------------------
// Reloads the cylindrical shell and links it to the higher level 
// obstacle in the obstacle tree
void BoxShell::reload( Obstacle& mother, istream& file )
{
  string ttag, buffer;
  
  // Read extra properties 
  file >> buffer >> m_lx >> m_ly >> m_lz >> m_shellWidth;
  
  // Standard Composite Obstacle reload
  if ( m_CompositeObstacle_id ) m_torsor.read( file ); 
  file >> ttag;
  while ( ttag != "</Composite>" ) 
  {
    ObstacleBuilderFactory::reload( ttag, *this, file );
    file >> ttag;
  }
  computeVolumeCenterOfMass();
  mother.append( this );
} 
