/**
 * Link check for ArborCore. Takes the address of each public entry point
 * so the static library is actually pulled in, and does not run the pipeline.
 */

#include "arbor.h"

int main()
{
  using namespace arbor;
  volatile auto ground = &segment::segment_ground;
  volatile auto instance = &segment::segment_instance;
  volatile auto semantic = &segment::segment_semantic;
  volatile auto dist = &segment::dist2root;
  volatile auto seeds = &seeds::find_seeds;
  volatile auto qsm_fn = &qsm::qsm;
  volatile auto qsf_fn = &qsm::qsf;
  volatile auto dtm_fn = &dtm::dtm;
  volatile auto hom = &utils::homogeneization;
  volatile auto sor_fn = &utils::sor;
  volatile auto aniso = &utils::anisotropy;
  volatile auto smooth = &utils::smooth3d;

  return ground && instance && semantic && dist && seeds && qsm_fn && qsf_fn && dtm_fn && hom && sor_fn && aniso && smooth ? 0 : 1;
}
