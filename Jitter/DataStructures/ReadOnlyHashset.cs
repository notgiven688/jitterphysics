using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter.Dynamics;
using System.Collections;

namespace Jitter.DataStructures
{

    public class ReadOnlyHashset<T> : IEnumerable, IEnumerable<T>
    {
        private HashSet<T> hashset;

        public ReadOnlyHashset(HashSet<T> hashset) { this.hashset = hashset; }

        public IEnumerator GetEnumerator()
        {
            return hashset.GetEnumerator();
        }

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return hashset.GetEnumerator();
        }

        public int Count { get { return hashset.Count; } }

        public bool Contains(T item) { return hashset.Contains(item); }

    }
}
