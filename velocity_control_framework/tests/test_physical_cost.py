import numpy as np
import pytest

from tools.physical_cost import realized


def test_realized_physical_cost_integrates_tracking_effort_and_slew_only():
    truth=np.zeros((2,9))
    reference=np.zeros_like(truth)
    truth[:,0]=[1.,2.]
    command=np.asarray([[.1,0,0,.4],[.2,0,0,.4]])
    result=realized(truth,reference,command,np.ones(9),np.ones(4),np.ones(4),
                    .4,np.array([.02,.02]),0.)
    assert result==pytest.approx([.10,.001,.0004,.1014])
