/** 
# Set of functions for a 2D circular cylinder 
*/


/** Tests whether a point lies inside the 2D circular cylinder */
//----------------------------------------------------------------------------
bool is_in_CircularCylinder2D_clone( const double x, const double y, 
	GeomParameter const* gcp )
//----------------------------------------------------------------------------
{
  return ( sqrt( sq( x - gcp->center.x ) + sq( y - gcp->center.y ) ) 
  	< gcp->radius );
}




/** Tests whether a point lies inside the 2D circular cylinder or any of its 
periodic clones */
//----------------------------------------------------------------------------
bool is_in_CircularCylinder2D( const double x, const double y, 
	GeomParameter const* gcp )
//----------------------------------------------------------------------------
{
  // Check if it is in the master rigid body
  bool status = is_in_CircularCylinder2D_clone( x, y, gcp );

  // Check if it is in any clone rigid body
  if ( gcp->nperclones && !status )
    for (int i = 0; i < gcp->nperclones && !status; i++)
    {
      GeomParameter clone = *gcp;
      clone.center = gcp->perclonecenters[i];
      status = is_in_CircularCylinder2D_clone( x, y, &clone );
    }

  return ( status );
}




/** Tests whether a point lies inside the 2D circular cylinder or any of its 
periodic clones and assign the proper center of mass coordinates associated to
this point */
//----------------------------------------------------------------------------
bool in_which_CircularCylinder2D( double x1, double y1, 
	GeomParameter const* gcp, vector* pPeriodicRefCenter, 
	const bool setPeriodicRefCenter )
//----------------------------------------------------------------------------
{
  // Check if it is in the master rigid body
  bool status = is_in_CircularCylinder2D_clone( x1, y1, gcp );
  if ( status && setPeriodicRefCenter )
    foreach_point( x1, y1 )
      foreach_dimension()
        pPeriodicRefCenter->x[] = gcp->center.x;

  //  Check if it is in any clone rigid body
  if ( gcp->nperclones && !status )
    for (int i = 0; i < gcp->nperclones && !status; i++) 
    {
      GeomParameter clone = *gcp;
      clone.center = gcp->perclonecenters[i];
      status = is_in_CircularCylinder2D_clone( x1, y1, &clone );
      if ( status && setPeriodicRefCenter )
        foreach_point( x1, y1 )
          foreach_dimension()
	    pPeriodicRefCenter->x[] = clone.center.x;
    }

  return ( status );
}




/** Computes the number of boundary points on the perimeter of the 2D circular 
cylinder */
//----------------------------------------------------------------------------
void compute_nboundary_CircularCylinder2D( GeomParameter const* gcp, int* nb ) 
//----------------------------------------------------------------------------
{
  double delta = L0 / (double)(1 << MAXLEVEL) ; 

  *nb = (int)( floor( 2. * pi * gcp->radius
	/ ( INTERBPCOEF * delta ) ) );
      
  if( *nb == 0 )
    printf( "nboundary = 0: No boundary points !!!\n" );
}




/** Creates boundary points and normal vectors of the 2D circular cylinder */
//----------------------------------------------------------------------------
void create_referenceRB_boundary_geomfeatures_CircularCylinder2D( 
	GeomParameter const* gcp,
	RigidBodyBoundary* dlm_bd, const int nsphere ) 
//----------------------------------------------------------------------------
{
  int k, m = nsphere;  
  double thetak, radius = gcp->radius;
  
  for (k = 0; k < m; k++) 
  {
    thetak = (double)(k) * 2. * pi / (double)(m); 
    dlm_bd->bp[k].x = radius * cos( thetak );
    dlm_bd->bp[k].y = radius * sin( thetak );
    dlm_bd->bp[k].z = 0.;
    // We arbitrary set the norm of the normal vector to 0.25 * radius
    dlm_bd->normal[k].x = ( dlm_bd->bp[k].x - gcp->center.x ) / 4.;
    dlm_bd->normal[k].y = ( dlm_bd->bp[k].y - gcp->center.y ) / 4.;
    dlm_bd->normal[k].z = 0.;         
  }
}




/** Finds cells lying inside the 2D circular cylinder */
//----------------------------------------------------------------------------
void create_FD_Interior_CircularCylinder2D( RigidBody* p, vector Index,
	vector PeriodicRefCenter )
//----------------------------------------------------------------------------
{
  GeomParameter const* gcp = &(p->g);
  Cache* fd = &(p->Interior);
  Point ppp;

  /** Create the cache for the interior points */
  foreach(serial)
    if ( in_which_CircularCylinder2D( x, y, gcp, &PeriodicRefCenter, true ) )
      if ( (int)Index.y[] == -1 )
      {
	ppp.i = point.i;
        ppp.j = point.j;		
        ppp.level = point.level;
	cache_append( fd, ppp, 0 );
        Index.y[] = p->pnum;
      }

  cache_shrink( fd );
}




/** Reads geometric parameters of the 2D circular cylinder */
//----------------------------------------------------------------------------
void read_reference_CircularCylinder2D( GeomParameter* gcp, 
	const double RotMat[3][3] ) 
//----------------------------------------------------------------------------
{    
  // TO DO
}




/** Update geometric parameters with the reference rigid body */
//----------------------------------------------------------------------------
void update_CircularCylinder2D_from_RBRef( GeomParameter* gcp, 
	RigidBody const* RBRef )
//----------------------------------------------------------------------------
{
  // Nothing to do  
}



/** Frees the geometric parameters of the 2D circular cylinder */
//----------------------------------------------------------------------------
void free_CircularCylinder2D( GeomParameter* gcp ) 
//----------------------------------------------------------------------------
{  
  // Nothing to do
}
